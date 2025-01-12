# dev-drivers

The dev-drivers repository serves as a catalogue of all the drivers created under the mcu-dev organization. These drivers are written based on Zephyr RTOS. 

>**Note**: You may refer to the [dev-apps](https://github.com/mcu-dev/dev-apps) repository for sample implementations of all the drivers created in this repository.

## Cloning the Repository

To fully clone this repository, including all its submodules, run the following commands:

To clone the repository and its submodules:
```
git clone --recurse-submodules https://github.com/mcu-dev/dev-drivers.git
```
You can also initialize and update the corresponding submodules if you have cloned the repository already:
```
git submodule update --init --recursive
```
Updating the submodule:

```
git submodule update --remote --recusrive
```

## License

Licensed under the MIT license.