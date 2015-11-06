:: Change HKCU to HKLM if you want to install globally.
:: %~dp0 is the directory containing this bat script and ends with a backslash.
REG ADD "HKCU\Software\Google\Chrome\NativeMessagingHosts\com.mrl.chrome.chalktalk.messaging" /ve /t REG_SZ /d "%~dp0com.mrl.chrome.chalktalk.messaging-win.json" /f
