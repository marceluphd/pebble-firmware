/* Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Use in source and binary forms, redistribution in binary form only, with
 * or without modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 2. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 3. This software, with or without modification, must only be used with a Nordic
 *    Semiconductor ASA integrated circuit.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import path from 'path';

import { remote } from 'electron';
import { logger, getAppDir } from 'nrfconnect/core';
import nrfjprog from 'pc-nrfjprog-js';
import SerialPort from 'serialport';

export const MODEM_KNOWN = 'MODEM_KNOWN';
export const MODEM_PROCESS_UPDATE = 'MODEM_PROCESS_UPDATE';
export const MODEM_WRITING_CLOSE = 'MODEM_WRITING_CLOSE';
export const MODEM_WRITING_END = 'MODEM_WRITING_END';
export const MODEM_WRITING_FAIL = 'MODEM_WRITING_FAIL';
export const MODEM_WRITING_READY = 'MODEM_WRITING_READY';
export const MODEM_WRITING_START = 'MODEM_WRITING_START';
export const MODEM_WRITING_SUCCEED = 'MODEM_WRITING_SUCCEED';

export const MODEM_DFU_NOT_STARTED = 'Not started.';
export const MODEM_DFU_STARTING = 'Starting...';

export const modemKnownAction = isModem => ({
    type: MODEM_KNOWN,
    isModem,
});

export const processUpdateAction = (
    message,
    percentage = 0,
    duration = 0,
    step = 0,
) => ({
    type: MODEM_PROCESS_UPDATE,
    message,
    percentage,
    duration,
    step,
});


export const writingReadyAction = fileName => ({
    type: MODEM_WRITING_READY,
    fileName,
});

export const writingCloseAction = () => ({
    type: MODEM_WRITING_CLOSE,
});

export const writingStartAction = () => ({
    type: MODEM_WRITING_START,
});

export const writingSucceedAction = () => ({
    type: MODEM_WRITING_SUCCEED,
});

export const writingFailAction = errorMsg => ({
    type: MODEM_WRITING_FAIL,
    errorMsg,
});

export const selectModemFirmware = () => dispatch => {
    remote.dialog.showOpenDialog(
        {
            title: 'Select a modem firmware zip file',
            filters: [{ name: 'Modem firmware zip file', extensions: ['zip'] }],
            properties: ['openFile'],
        },
        fileName => {
            if (fileName && fileName.length > 0) {
                dispatch(writingReadyAction(fileName[0]));
            }
        },
    );
};


export const cancelUpdate = () => dispatch => {
    dispatch(writingCloseAction());
};

export const performJlinkUpdate = (serialNumber, fileName) => dispatch => {
    const progressCallback = progress => {
        logger.info(progress.process);
        dispatch(processUpdateAction(progress.process));
    };

    const callback = err => {
        if (err) {
            logger.error(`Modem DFU failed with error: ${err}`);
            let errorMsg = err.message || err;
            if (err.message.includes('0x4')) {
                errorMsg = 'Failed with internal error. '
                    + 'Please click Erase all button and try updating modem again.';
            }
            dispatch(writingFailAction(errorMsg));
            return;
        }
        logger.info('Modem DFU completed successfully!');
        dispatch(writingSucceedAction());
    };

    nrfjprog.programDFU(serialNumber, fileName, progressCallback, callback);
};

export const performMcuUpdate = fileName => (dispatch, getState) => {
    const { port, port2 } = getState().app.mcuboot;
    const mcubootFilePath = path.resolve(
        getAppDir(),
        'resources',
        'firmware',
        'nrf9160_pca20035_firmware_upgrade_app_0.1.0.hex',
    );
    const noSerialNumber = -1;
    const baudrate115k2 = 115200;
    const baudrate1m = 1000000;
    const timeout = 12000;
    let totalPercentage = 0;
    let totalDuration = 0;
    let progressMsg = '';
    let step = 1;

    const progressCallback = progress => {
        let dfuProcess;
        try {
            dfuProcess = JSON.parse(progress.process);
        } catch (e) {
            dispatch(processUpdateAction(
                progress.process,
                totalPercentage,
                totalDuration,
                step,
            ));
            return;
        }

        if (dfuProcess && dfuProcess.operation === 'upload_image') {
            totalPercentage = dfuProcess.progress_percentage;
            totalDuration = dfuProcess.duration;

            progressMsg = dfuProcess.message || progressMsg;
            dispatch(processUpdateAction(
                progressMsg,
                totalPercentage,
                totalDuration,
                step,
            ));
        }
    };

    const modemCallback = err => {
        if (err) {
            logger.error(`Modem DFU failed with error: ${err}`);
            let errorMsg = err.message || err;
            if (err.message.includes('0x4')) {
                errorMsg = 'Failed with internal error. '
                    + 'Please click Erase all button and try updating modem again.';
            }
            dispatch(writingFailAction(errorMsg));
            return;
        }
        logger.info('Modem DFU completed successfully!');
        dispatch(writingSucceedAction());
    };

    const mcuCallback = err => {
        // logger.info('mcuCallback  start');
        if (err) {
            let errorMsg = err.message || err;

            // Program without setting device in MCUboot mode will throw such an error:
            // Errorcode: CouldNotCallFunction (0x9)
            // Lowlevel error: Unknown value (ffffff24)
            // Interpret the error in a way that user can understand it easily.
            if (errorMsg.includes('0x9')) {
                errorMsg = 'Please make sure that the device is in MCUboot mode and try again.';
            }

            logger.error(`MCUboot DFU failed. ${errorMsg}`);
            dispatch(writingFailAction(errorMsg));
            return;
        }
        // logger.info('modemUartDfu  strat', port2);

        const serialPort = new SerialPort(port, {
            baudRate: 115200,
            autoOpen: false,
        });
        const err1 = serialPort.open();
        if (err1) {
            logger.info('IsOpen:', serialPort.isOpen);
            logger.info('err:', err1);
        } else {
            const buf1 = Buffer.from('060941417343414141424141426D4261446745773D3D0A', 'hex');
            serialPort.write(buf1);
            serialPort.write(buf1);
            serialPort.close();
        }
        const modemUartDfu = () => nrfjprog.programModemUartDFU(
            noSerialNumber,
            fileName,
            port2,
            baudrate1m,
            timeout,
            progressCallback,
            modemCallback,
        );
        // logger.info('modemUartDfu  end');
        step += 1;
        setTimeout(modemUartDfu, 2000);
    };
    logger.info('programMcuBootDFU  strat', port);
    nrfjprog.programMcuBootDFU(
        noSerialNumber,
        mcubootFilePath,
        port,
        baudrate115k2,
        timeout,
        progressCallback,
        mcuCallback,
    );
};

export const performUpdate = () => (dispatch, getState) => {
    dispatch(writingStartAction());
    dispatch(processUpdateAction(MODEM_DFU_STARTING));
    logger.info('Modem DFU starts to write...');

    const { modemFwName: fileName } = getState().app.modem;
    const serialNumber = Number(getState().app.target.serialNumber);
    logger.info(`Writing ${fileName} to device ${serialNumber || ''}`);


    if (getState().app.mcuboot.isMcuboot) {
        dispatch(performMcuUpdate(fileName));
    } else {
        dispatch(performJlinkUpdate(serialNumber, fileName));
    }
};
