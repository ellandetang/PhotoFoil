# PhotoFoil
A codebase to facilitate the extraction of propeller geometry using photogrammetry

The included code is an executed example of the code on a sample set of data produced by COLMAP found in "dense"

General workflow is to start by setting the overall properties in propProperties.m
Then use fitPrism.m to load, align, and scale the pointcloud data. In this script, 
the seed points for the fitting prism can be selected from the figure and exported. For this example,
the seed points are hard coded in the propProperties.m file. The prism dimensions are also hard coded 
under the "define known figure" header.

From there, analyzeCloud can be used to trim the dataset and filter out some of the bad data points.
Color filtering is done by changing values of the colorSample and colorThreshold variables. 

Then, either fitFoilAutomatic.m or fitFoilManual.m can be used to extract the propeller parameters.
User input shoud be indicated by a message in the console.
