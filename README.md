## General Info
* If you use this code, please cite the following:
```
@article{ardon2021affordanceAware,
  title={Affordance-Aware Handovers with Human Arm Mobility Constraints},
  author={Ard{\'o}n, Paola and Cabrera, Maria, and Pairet, {\`E}ric and Petrick, Ronald PA and Ramamoorthy, Subramanian and Lohan, Katrin S and Cakmak, Maya},
  journal={IEEE Robotics and Automation Letters with presentation on ICRA},
  year={2021},
  publisher={IEEE},
  doi = {10.1109/LRA.2021.3062808}
}
```
* Complementary [video](https://youtu.be/cFsAEpSn_LI) with experiments

## Pre-requisites

* The hand pose is detected using [Pose-REN](https://github.com/xinghaochen/Pose-REN)
* The reasoning knowledge base is structured on the [pracmln](http://pracmln.org/) toolbox

## To run a demo on real scenarios

* Detect the user hand (in accordance to the model used in Pose-REN). We are using the real sense camera, as such we use the `realsense_realtime_demo_librealsense2.py` from Pose-REN

* Using the hand pose from Pose-REN we calculate the costs using `plot_costs/main_costs.py`. If wanting to visualise safety cost alone: `main_costs.py -r`. For visualisation on reachability cost alone: `main_costs.py -s`

## To run a demo on simulated scenarios

* Launch the human mannequin: `roslaunch human_model_gazebo view_human.launch`
* Select an object from the examples or create your own object to handover and put it in the example folder `objects_to_handover`
* Use the same files as for the real demo to detect the hand pose and calculate the costs

## To detect affordances for handover

* We use our previous framework to detect the objects semantics that are associated to recognise an [affordance](https://github.com/PaolaArdon/grasp_affordance_reasoning_demo)
* If using the SRL, in the pracmln toolbox use the 70% of dataset in `mln/dbs` or you can find the trained model in `mln/mlns`
