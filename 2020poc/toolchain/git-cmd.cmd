@ECHO OFF
CALL "%~dp0\cmd\env.cmd"
CD "%ZEPHYR_BASE%\.."

IF NOT EXIST ".west\config" (
    echo NCS repositories are not yet cloned here. To do so run the following command:
    echo.
    echo git-bash -c "ncsmgr init-ncs"
    echo.
)

START /B /WAIT CMD
EXIT
