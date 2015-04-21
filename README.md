# p5glove
OS/X Version of p5glove software that works with yosemite

This is a mashup of Rajmil Fischman's MAES work: (see software tab) http://www.keele.ac.uk/music/people/rajmilfischman/

The original code that Rajmil used:
https://github.com/ezrec/libp5glove

And an update to the Mac USB code by a slight modification to this library:
https://github.com/signal11/hidapi

To Do:
* unfortunately I had to modify the signal11 software. It returns two USB devices, one that has 3 bytes, and 1 that is 24 bytes. We want the 24 byte one, so I put a hack in there to ignore the interface if it doesn't have more than 5 bytes.
* i was lazy when I integrated the usb library - i should have given it the same interface as the libp5glove generic one, so the windows code could share the non-mac specific things. This is a quick fix
