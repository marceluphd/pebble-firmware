@ECHO OFF

pushd "%~dp0\.."
SET tcdir=%CD%
popd

SET PATH=%tcdir%;%tcdir%\mingw64\bin;%tcdir%\opt\bin;%tcdir%\opt\bin\Scripts;%tcdir%\segger_embedded_studio\bin;%PATH%
SET PYTHONPATH=%tcdir%\opt\bin;%tcdir%\opt\bin\Lib;%tcdir%\opt\bin\Lib\site-packages
SET Python3_ROOT=%tcdir%\opt\bin
SET GNUARMEMB_TOOLCHAIN_PATH=%tcdir%\opt
SET ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
SET NCS_TAG=v1.3.0

pushd "%tcdir%\.."
SET ZEPHYR_BASE=%CD%\zephyr
popd
