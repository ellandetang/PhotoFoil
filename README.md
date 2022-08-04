# PhotoFoil
A codebase to facilitate the extraction of propeller geometry using photogrammetry

General workflow is to start by using fitPrism.m to load, align,and scale the pointcloud data.
From there, analyzeCloud can be used to trim the dataset and filter out some of the bad data points.
Then, either fitFoilAutomatic.m or fitFoilManual.m can be used to extract the propeller parameters.
