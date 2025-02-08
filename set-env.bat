@echo off
setlocal EnableDelayedExpansion

:: Check if .env file exists
if not exist ".env" (
    echo Error: No .env file found
    exit /b 1
)

echo Reading .env file...

:: Read and parse .env file
for /f "tokens=1,2 delims==" %%a in (.env) do (
    set key=%%a
    set value=%%b

    :: Trim whitespace
    set key=!key: =!
    set value=!value: =!

    echo Processing: !key! = !value!

    :: Set using setx (user level, no elevation needed)
    setx !key! !value!

    :: Set for current session too
    set "!key!=!value!"

    :: Verify
    echo Environment variable !key! is set to: !value!
)
