# picar

## First steps
Install packages from **requirements.txt** -  e.g. under a venv

## Starting the server components

Run as root if it does not work with normal user privilages.

First, use the prepared python3 venv:
```bash
source .venv/bin/activate
```

Launch the picar server in a screen:
```bash
screen -S server
./start_cs.sh
```
Detach the screen by pressing Ctrl+a then d.

Launch the bridge server that is the backend of the simple UI:
```bash
screen -S bridge
./start_br.sh
```
Detach the screen by pressing Ctrl+a then d.

## The experimental setup

<p align="center">
<img src="images/IMG_20260225_164854.jpg" title="Experimental Setup">
<p/>
