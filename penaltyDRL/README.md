#  Deep Reinforcement Learning in a Humanoid Robot to learn shoot penalty kick


This repository presents the DRL source code that made the learning of the images from the Webots simulator,
the Matthias Plappert was used as a base, available in [https://github.com/matthiasplappert/keras-rl](https://github.com/matthiasplappert/keras-rl)


## Install keras

Installing `keras-rl`. Just run the following commands and you should be good to go:
```bash
pip install keras-rl
```
This will install `keras-rl` and all necessary dependencies.


You'll also need the `h5py` package to load and save model weights, which can be installed using
the following command:
```bash
pip install h5py
```

## How do I get started?

Once you have installed everything, you can try out a simple example:
```bash
python examples/duel_dqn_goalkeeper.py
```

## Requirements
- Python 2.7
- [Keras](http://keras.io) >= 1.0.7

That's it. However, if you want to run the examples, you'll also need the following dependencies:
- [OpenAI Gym](https://github.com/openai/gym)
- [h5py](https://pypi.python.org/pypi/h5py)



