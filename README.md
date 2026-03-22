<h1 align="center">boy - gameboy emulator</h1>

<p>this is a learning exercise for me to learn the inner workings of a computer</p>

<h2>progress</h2>
<ul>
    <li>currently passes 11/11 of the [blargg](https://github.com/retrio/gb-test-roms) cpu test roms</li>
    <li>for testing each opcode, [gb doctor](https://github.com/robert/gameboy-doctor) was used to compare the cpu state after each cycle with the state of a working emulator</li>
    <li>currently only mbc1 is implemented but all memory maps have been implemented</li>
</ul>

<h2>todo</h2>
<ul>
    <li>ppu</li>
    <li>gui for graphical debugging and playing game (duh)</li>
    <li>save states</li>
    <li>link cable emulation (much harder than i thought since link cable is much much faster than any network connection possible)</li>
    <li>implement more mbcs</li>
</ul>

<p>in the .zed folder there are tasks to run, debug and test the emulator, they work in the zed ide but can easily but typed into command line if u use something else</p>

<p>prs are welcome, please dont use ai since it can easily one shot this and it would be nice to be able to use this for learning</p>



