# NanoArtNet
This is a review in order to implement an ArtNet node through Arduino Nano and ENC28J60 Nano shield. Right now, only one universe can be sent by DMX interface.

## Dependencies ##
This sketch needs the following libraries:  
* [EtherCard](https://github.com/njh/EtherCard)  
* [DmxSimple](https://github.com/PaulStoffregen/DmxSimple)
	
You can find these in the Arduino IDE _Library Manager_.

NOTE: The repository includes a modified version of EtherCard library in order to save RAM.

## Art-Net protocol ##
[Specification for the Art-Net 4 protocol](http://artisticlicence.com/WebSiteMaster/User%20Guides/art-net.pdf)
