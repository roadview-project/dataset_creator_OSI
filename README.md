# Dataset Creator OSI

this package exists to make usage of a rosbag data collection and convert it to OSI files.

first step is to create a yaml descriptor file of your sensors, to do that use config_creator.py

The yaml files describe how the datasetcreator file will work.

it is of great importance to have the names of the classes equal to those created in the package. if the type is a Camera, put Camera in type. not camera, or cAmeRA.

The main file is dataset_creator.py

when executed it will generate the osi files in the specified folder from the specified rosbag.

Suggestion, check out the [parallel project](https://doi.org/10.5281/zenodo.1146014) and usage, it is quite benefitial when working with more than one bag.

# Citation

```
@Journal{
  title = {REHEARSE: adveRse wEatHEr datAset for sensoRy noiSe modEls},
	author = {Poledna,Yuri and Drechsler, Maikol Funk and Valentina Donzella and Pak Hung Chan and Pierre Duthon  and Huber, Werner},
	booktitle = {Intelligent Vehicles Symposium 2024 - Jeju Island South Korea},
	year = {},
	volume = {},
	number = {},
	pages = {},
	doi = {waiting publishing to be here :)},
}
```

# Acknoledgment

Co-Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.

UK and Swiss participants in this project are supported by Innovate UK (contract no. 10045139) and the Swiss State Secretariat for Education, Research and Innovation (contract no. 22.00123) respectively.
