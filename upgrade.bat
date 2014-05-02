@echo off
echo Providing a command-line example of the script needed to upgrade
rem vomus ?
msiexec /i Ryan_Debug.msi ADDLOCAL=ALL REINSTALL=ALL REINSTALLMODE=omus

