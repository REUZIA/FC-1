# Barometric timer

This timer provide two modes. By default, it run the normal mode which act as a barometric timer and log the sensor data into its file system. But by connecting the pin .. to 3v3 before powering the board, it run the CLI mode. In this second mode you can run multiple commands via a Serial connection (or use Python script to extract the file data).

List of commands and there descriptions

Command Name|Bytes|Description
-|-|-
Info|'i'|Information about the file system.<br><pre>f=NumberOfFile<br>Index:a=StartAdress\|s=SizeOfFile<br>[...]<br>b=AddressOfNextFreePage</pre>
Delete|'d'|Reset file system by writing all zeros to the last sector where files informations are stored. System will act as if nothing was on the flash and new files will override previous data.
New file|'n'|Manually create a new file of size 0. For test purposes mostly.
Write|'w'|Will write a buffer of 256 bytes to the last file full of one letter ASCII code starting from 'a'. And then each time you make this command the letters writes are increment by one. For example, first run will write 256 'a' and then 256 'b', 256 'c', etc..
Erase|'e'|Erase first sector of file system (e.g. second sector before flash end)
Read|'r'|Read last file and send its content.
Read file|['r',IndexOfFile]|Read a specific file and send its content. The index is expected to be an unsigned 8bit number starting from 0. The two bytes should be send at the "same time".
Test sequence|'t'|Run a test sequence. It will erase the second last sector of flash one sector before file system file storage and write 16 chunk of 256 letters. Then, it will read the sector to unsure that writing was successful. WARNING! Running this test will destroy the first file so be sure to run it without precious informations inside the flash.
Quit|'q'|Trigger a reset of the board
