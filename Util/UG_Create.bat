@echo off
REM ***********************************************************************
REM This script uses pandoc to convert all of the markdown readme files to
REM html files. In order to preserve the links between the files, it requires
REM reading in each .md file and changing the links to link to *.html from 
REM *.md. In order to do this, it creates a temporary copy of each markdown 
REM file, converts that to html, and then deletes the temporary .md file
REM Finally, it renames the top level README.html to BAM_Users_Guide_v1p0
REM
REM Created by Michael J. Acheson, NASA Langley Research Center, 6/27/2025
REM michael.j.acheson@nasa.gov
REM ***********************************************************************

setlocal enableDelayedExpansion

set origdir = %CD%
set "sourcedir=../"

FOR /R "%sourcedir%" %%f IN (*.md) DO (
	echo Processing: %%f
	
	REM create a new temporary file to process
	set "newfile=%%f"
	set "newfile=!newfile:~0,-3!"
	set "newfile=!newfile!_temp.md"
	echo !newfile!
	set newline=^&echo.

	REM Now replace .md with .html within each README file (this will get the links to work)
	FOR /F "tokens=1,* delims=:" %%A in ('findstr /N "^" "%%f"') DO (
		set "line=%%B"
		IF "!line!" == "" (
			echo:>>!newfile!
		) else (
			set "line=!line:.md=.html!"
			echo !line!>>!newfile!
		)
	)

	REM create a new outfile name (html)
	set outfile=
	set "outfile=%%f"
	set "outfile=!outfile:~0,-3!"
	set "outfile=!outfile!.html"

    REM use pandoc to render the markdown files at html
	pandoc --mathml -s --css pandoc.css -f markdown -t html "!newfile!" -o "!outfile!"
    
	REM now delete the temporary md file
	del !newfile!
)

echo Completed Conversions!

REM Once complete, take the toplevel README.html and rename as BAM_Users_Guide_V1p0.html
REM echo Renaming the top level README.html file...

REM cd %origdir%
REM cd %sourcedir%   
REM move /y "README.html" "BAM_Users_Guide_v1p0.html"