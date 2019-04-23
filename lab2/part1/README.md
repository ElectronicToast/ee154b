### Usage

Run in virtual env with instructions in README in lab2/.

```
$ py loopback.py
```

The program then prompts you to type upper or lowercase letters to send to the Arduino for a loopback test. The character sent will be incremented (with wraparound) then sent back. If a non-letter is sent, an exclamation point will be sent back "!".