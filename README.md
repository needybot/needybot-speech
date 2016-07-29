Needybot Speech
==================

Open Source speech client for Needybot.

This package uses [Ivona](https://www.ivona.com/) for raw speech generation, a
free developer account is required.

Getting started
---------------

All apt-get and python (pip) dependencies are installed via a command in [CMakeLists.txt](CMakeLists.txt). Currently, the dependencies required are:

- ladspa-sdk
- libsox-fmt-all
- python-pygame
- sox
- tap-plugins

And in requirements.txt:

- pydash
- pyvona
- requests

This project expects you will be working within a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Assuming you have a catkin workspace set up in your home directory, you can get up and running by doing the following:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/needybot/needybot-speech.git
$ cd ~/catkin_ws
$ catkin_make
```

If all does not go well, please read our [contributing guidelines](CONTRIBUTION_GUIDELINES) and create an [issue](https://github.com/needybot/needybot-speech/issues).

Environment Variables and ROS Parameters
----------------------------------------

### ENV

The following [Ivona](https://www.ivona.com/) environment variables must be set for this package to work:

- `IVONA_ACCESS_KEY`
- `IVONA_SECRET_KEY`

### ROSPARAM

- `/needybot/speech/voice/name`: string [`'Justin'`]
  - The voice identifier for Ivona
- `/needybot/speech/voice/speech_rate`: string [`'medium'`]
  - Uses standard SSML prosody rate labels
    1. x-slow
    2. slow
    3. medium (default)
    4. fast
    5. x-fast
    6. default (medium)
- `/needybot/speech/voice/codec`: string [`'ogg'`]
  - The audio encoding format
    1. mp3
    2. mp4
    3. ogg
    4. speechmark
- `/needybot/speech/voice/cache_dir`: string [`'/path/to/needybot_speech/cache'`]
  - The path at which your cache manifest and audio files will live.
- `/needybot/speech/voice/clear_cache`: boolean [`false`]
  - Flag signaling that you would like to _completely_ clear your speech cache.
- `/needybot/speech/voice/warm_cache`: boolean [`true`]
  - Flag signaling that you would like to prewarm the speech cache.

Adding Dialog
-------------

By default, all dialog is stored in param/dialog_manifest.yml, however, you may override this when including [launch/needybot_speech.launch](launch/needybot_speech.launch) in your own file with the `<arg name="manifest" value="/abs/path/to/my_dialog_manifest.yml" />` tag. All dialog will be loaded into the rosparam server under the `/needybot/speech/dialog` namespace. For example, your manifest might look like this:

```yaml
just_hello:
  template: "Hello!"
introduction:
  template: "Hello world! My name is ${name} and my favorite number is ${favorite_number}."
  params:
    name: "Needybot"
    favorite_number: "pi"
```

In the first entry labeled `just_hello`, Needybot will be fed the template and speak it normally with no substitutions (substitutions are optional). In the second entry labeled `introduction`, we see a template with placeholder identifiers. We use the `Template` class provided by the `string` module in python's standard library. Beneath the template is a dictionary of default parameters, should you not provide any at run time.

TODO: action server example

Contributing
------------

See our contribution guidelines [here](CONTRIBUTION_GUIDELINES.md).

Testing
-------

*WARNING*: Running package tests will clean and warm your dialog cache directories. Keep this in mind when working with production code and be sure to backup your data when necessary.

This package uses the `rostest` command for testing, to run tests:

```
$ rostest needybot_speech needybot_speech.test
```

License
-------

This project is release under Apache 2.0. Please see the [LICENSE](LICENSE) file for more details.

