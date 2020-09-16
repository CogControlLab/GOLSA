LICENSE
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.


TO USE
To see the model in action, first run some of the sample simulations from golsaMaster.m. Each section runs a script in the simulations folder which in turn calls a script to set up the network before running it. The network is constructed out of objects defined in the components folder and operates on state spaces stored in the stateSpaces folder.

The structure of each network, details of the simulated run, and display of simulated data can be modified within the appropriate files. Some of the flags and option should be clear based on their name and associated comments. A few of the less clear options are described below.

At the end of each simulation, a multifaceted plot shows the timecourse of activity for a selection of layers (the timecourse of projection weights can also be displayed). To change which components are plotted in this way, add or remove the relevant names (as strings) from the “plotItems” cell array. The network method gatePlot shows the activities of two layers connected by a particular projection, the activity of a node which gates the projection, and the opening and closing times of the projection. 

If the DISPLAY flag is set to 1, some activities or weights will be displayed in real time, determined by the network set_display method. 




