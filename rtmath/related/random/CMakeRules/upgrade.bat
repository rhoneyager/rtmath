@echo off
echo Providing a command-line example of the script needed to upgrade
rem vomus ?
msiexec /i @MSI_NAME@.msi ADDLOCAL=ALL REINSTALL=ALL REINSTALLMODE=vomus

