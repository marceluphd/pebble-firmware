@ECHO OFF
CALL "%~dp0\cmd\env.cmd"
CD "%ZEPHYR_BASE%\.."

START /B emStudio.exe
