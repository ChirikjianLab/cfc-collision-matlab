# "/data" folder
This folder stores the benchmark data obtained from the C++ implementation. 

## Running instruction
To visualize the benchmark results in both static and continuous cases, please:

+ Run C++ benchmark code and obtain all the `.csv` files for different types of geometric pairs. Each type should be stored in separate folder, with name `${GeomType}-${GeomType}` (`${GeomType} = {SQ, E, PE}`).
+ Put all the folders for different geometric types into a new folder with an arbitrary name (the default choice uses date as the folder name). And put the folder into either `/static` or `/continuous` folder generated in this directory accordingly.
+ Run the MATLAB script `/test/parse_benchmark_data.m` to parse the raw benchmark data. The parsed `.mat` files will be stored in `/static/data_parsed/` or `/continuous/data_parsed/` folder. Please change the highlighted parameters on the top of the script accordingly. Parameters:
	+ `opt`: choose from `static` and `continous`
	+ `date`: change accordingly to the name of the folder including all the benchmark data (specified in the previous step).
+ Run the MATLAB script `/test/demo_benchmark_data.m` to visualize the comparison results for the running time of different solvers.

### Demonstrated data for results in the paper
For a demonstration (results presented in the paper), please simply download the data from the [Google drive](https://drive.google.com/drive/folders/17jSSC-EIhiSTqXSgfoEOs4R7mzKy1d1i?usp=sharing), and copy-paste the folders `/static` and `/continuous` from the `/benchmark` folder into the current directory. Then, running the last step of the instruction will display the results.
