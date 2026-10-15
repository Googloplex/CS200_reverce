@echo off
for /d %%J in ("C:\Program Files\Eclipse Adoptium\jdk-21*") do set "JAVA_HOME=%%J"
set "_JAVA_OPTIONS=-Duser.home=C:\tools\ghidra_home -Djava.io.tmpdir=C:\tools\ghidra_home\tmp"
set "USERPROFILE=C:\tools\ghidra_home"
set "APPDATA=C:\tools\ghidra_home\AppData\Roaming"
set "LOCALAPPDATA=C:\tools\ghidra_home\AppData\Local"
set "TEMP=C:\tools\ghidra_home\tmp"
set "TMP=C:\tools\ghidra_home\tmp"
call "C:\tools\ghidra_12.1.4_PUBLIC\support\analyzeHeadless.bat" "C:\tools\re" servo_fw -import "C:\tools\re\servo_01_flash.bin" -processor ARM:LE:32:Cortex -loader BinaryLoader -loader-baseAddr 0x08000000 -overwrite -scriptPath "C:\tools\re\scripts" -preScript ServoPre.java -preScript ServoSvd.java "C:\tools\re\LCM32F037.svd" -postScript ServoDecomp.java "C:\tools\re\out" < nul
