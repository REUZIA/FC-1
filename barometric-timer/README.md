## Barometric Timer

## Normal mode

## CLI mode

CLI mode allow you to easily interact with onboard filesystem.

Commands|Description
--|--
ls $DIR_PATH|Allow you to inspect the content of a directory on disk. If no dir is provided then root "/" is used.
rm $FILE_DIR_PATH ...|Allow you to delete a file or an entire directory from disk. You could delete multiple entry at once by chaining them and separate them with a whitespace. If no args are provided the command will return an error.
cat $FILE_PATH|Allow you to display the raw content of a file. A valid file path is expected.
test|Runs a sequence of unit test and output the result.
exit|Reset the microcontroller.

