1. Have ubuntu installed and updated
2. Have openrave installed from. Follow instructions from https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html
3. Go to arm control folder
4. Ensure all con.get statements are commented so it doesn't try to communicate with the rover. (Tell sendMessage to return 0 for the new code)
5.Run script with ./filename.ext
6. If it's not an executable, ensure "#!/Use/bin/env python" is at the top of the file and type "chmod +x filename.ext" in the terminal
7. Try 5 again.
