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
@INPROCEEDINGS{10588491,
  author={Poledna, Yuri and Drechsler, Maikol Funk and Donzella, Valentina and Chan, Pak Hung and Duthon, Pierre and Huber, Werner},
  booktitle={2024 IEEE Intelligent Vehicles Symposium (IV)}, 
  title={REHEARSE: adveRse wEatHEr datAset for sensoRy noiSe modEls}, 
  year={2024},
  volume={},
  number={},
  pages={2451-2457},
  keywords={Point cloud compression;Rain;Laser radar;Noise;Radar;Open systems;Sensor phenomena and characterization},
  doi={10.1109/IV55156.2024.10588491}}

}
```

# Acknoledgment
Co-funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. Project grant no. 101069576.
UK participants in this project are co-funded by Innovate UK under contract no.10045139. 
Swiss participants in this project are co-funded by the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract no. 22.00123.

