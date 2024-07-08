# Dancing MiRo project
## This project is a dancing mood that enables MiRo to dance while it streams music <br>
### To run the code follow the following steps:

Go to the **dancing_miro_jh directory**<br>

1 - install the required libraries using the command `pip install -r requirements.txt`

2 - run `roslaunch dancing_miro_jh full_program.launch dancing_mode:= {MODE_NAME}` 

MODE_NAME can either be **"Spotify"** or **"Auto"**, if an invalid name is entered, Auto Mode will run automatically.

**NOTE:** It is possible that sound localisation for getting MiRo to face the music will not work in a loud environment and may result in him trying to face a lot of directions at once. 
If this is the case comment out the **lines 359** and **360**, then this step will be skipped.


