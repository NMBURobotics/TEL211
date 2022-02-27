# TEL211 Assignments 3
Refer to these example codes while working on the assignment 3.


## Text to speech example

A simple python script to convert a string to speech. 
Install required packages with ;

```bash
 sudo apt install python3-pip
 sudo apt install mpg321
 sudo pip3 install gTTS
```

Then just `cd` into correct folder and do; 

```bash
python3 text_to_speech.py
```

## ROS services example
In this example package(`service_server_client_pkg`), we create a service server where you provide a directory path, and the server returns you all folders in this directory. 


See the service definition with; 
```bash
rossrv show service_server_client_pkg/ListFolders
```

You should see; 
```bash
string path
---
string[] folders
```

