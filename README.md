<h1 align="center">boy - gameboy emulator</h1>

this is a learning exercise for me to learn the inner workings of a computer

## progress
* currently passes 9/10 of the [blargg](https://github.com/retrio/gb-test-roms) cpu test roms, since i have not fully implemented interrupts yet
* for testing each opcode, [gb doctor](https://github.com/robert/gameboy-doctor) was used to compare the cpu state after each cycle with the state of a working emulator
* currently only mbc1 is implemented but all memory maps have been implemented

## todo
* implement interrupts, just need to add the handlers, they can already be requested and cancelled
* ppu
* gui for graphical debugging and playing game (duh)
* save states
* link cable emulation (much harder than i thought since link cable is much much faster than any network connection possible)
* implemented more mbcs

in the .zed folder there are tasks to run, debug and test the emulator, they work in the zed ide but can easily be typed into command line if u use something else

prs are welcome, please dont use ai since it can easily one shot this and it would be nice to be able to use this for learning
