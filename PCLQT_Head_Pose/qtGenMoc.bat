@ECHO ON
FOR %%f IN (*.h) DO moc.exe %%f -o moc_%%~nf.cpp

FOR %%f IN (*.ui) DO uic.exe %%f -o ui_%%~nf.h