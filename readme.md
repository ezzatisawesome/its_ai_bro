Hello!

This is Vale's, Malek's, Nathania's, and Ezzat's group.

Just use AI bro.

Section Handout Below:

# Using `git`

Git is a source control tool that allows people to share code with each other, and more importantly, keep track of a history of code changes for any type of software development. We will guide you through creating a Git repository and share with you some common workflows when working collaboratively with multiple people on the same repository.

**Task 2.1 – Create a GitHub account.** Go to [Github](https://github.com) and create an account with your

university email if you have not yet done so before.

**Task 2.2 – GitHub authentication.** Use the following command and follow the prompts to authenticate the terminal with your GitHub account.

```bash
gh auth login
```

**Task 2.3 – Setup a GitHub repository.** Create a private new repository with the name `<repo>` you picked earlier. Follow the guide on you newly created GitHub project page to initialize `∼/autonomy ws/src/<repo>` as a git repository and push the README to GitHub.

**Task 2.4 – Add collaborators.** Go to project setting in GitHub and add your teammates as collaborators to the repository you just created.

**Task 2.5 – Add group info by creating a PR.** Use the following steps as a guide

1. Create a new branch.
2. Add a new file named `team.txt` and edit it to include all SUNetIDs of your team members (separated by newlines).
3. Add, commit, and push the new file to a new branch.
4. Create a pull request from the GitHub project page.
5. Review the changes and merge it back to `main` branch.

**Checkpoint**

### Cheat Sheet

```bash
# local commands
git checkout <branch>     # switch to <branch>
git checkout -b <branch>  # create a new local branch from the current branch
git add .                 # add all files in the current directory
git add <file or dir>     # add a single file or directory
git add -u                # add all tracked files
git commit -m "<msg>"     # commit all added changes to the local branch

# server commands
git fetch             # sync remote branch (does not change local files)
git pull              # sync remote branch and also update local files
git clone <url>       # clone git repo from <url> to the current directory
git push -u origin <branch>  # push to a newly created local branch and track remote branch
git push                     # subsequent pushes after a local branch is tracked
```

# Executables

## Python

All executables are just files with the `x` permission turned on. Here we focus on two scripting languages, Python and Shell, which you will use the most to build up your robot autonomy stack.

**Task 3.1.1 – Create a Python executable.**

1. Create a Python file at `<repo>/scripts/section1.py`.
2. Edit the file and use the following code as a start-up template.
    
    ```python
    #!/usr/bin/env python3
    
    # add import and helper functions here
    
    if __name__ == "__main__":
        # code goes here
    ```
    
    The first line, a.k.a the shebang, is always required for the OS to find the correct Python interpreter to execute this file. Shell scripts will require a different shebang (see [Shell](https://www.notion.so/Section-1-UNIX-Git-Python-ROS-25-26-27a66c3c12a881d7ab6bf03f6f609f8a?pvs=21)).
    
3. Change the permission of the file to make it an executable.
    
    Hint: use `chmod`.
    
4. Check that the permission of the file is indeed correct by running the following command
    
    ```bash
    ls -l <path to section1.py>
    ```
    
    and you should get a line starting with
    
    ```bash
    -rwxr-xr-x # ...
    ```
    
    where the `x` indicates that the execute permission is indeed turned on for the file.
    

**Task 3.1.2 – Execute it!** Add something simple to the file (e.g. `print("hello world")`), and then try to execute the file by writing the full path to `section1.py` and hit return.

Note: if you are currently in the `<repo>/scripts` directory in your terminal, you cannot just type `section1.py` and run the code. Instead, you need to type `./section1.py`.

**Task 3.1.3 – Learn about numpy matmul operator.** Import numpy as `np` in your Python script and add the following snippet to the main block to generate two random matrices.

```python
np.random.seed(42)
A = np.random.normal(size=(4, 4))
B = np.random.normal(size=(4, 2))
```

Write code to compute and print out the matrix product A · B using the Python matmul operator – `@`. The result should match the following

```python
array([[ 0.67459114, -1.96480447],
       [ 2.81615463, -1.19285965],
       [-0.72781678, -0.14561428],
       [-1.07385636,  3.96873247]])
```

**Checkpoint**

# ROS Sensor Data Visualization

1. Launch in two terminals:
    1. First run
        
        ```bash
        ros2 launch asl_tb3_sim root.launch.py
        ```
        
    2. In the other terminal, run
        
        ```bash
        ros2 topic list
        ```
        
        You should see the following topics:
        
        ```bash
        /clock
        /cmd_vel
        /imu
        /joint_states
        /map
        /map_metadata
        /odom
        /parameter_events
        /pose
        /robot_description
        /rosout
        /scan
        /sim/pose
        /slam_toolbox/feedback
        /slam_toolbox/graph_visualization
        /slam_toolbox/scan_visualization
        /slam_toolbox/update
        /tf
        /tf_static
        ```
        
    
    2. Visualize Sensor Data
    
    ```markdown
    rviz2
    ```
    

**Task 4.1.1 – Visualize Sensor Data**

Add:

- Laser scan display → `/scan`
- Odometry → `/odom`
- Map → `/map`

![image.png](attachment:48441117-18e0-4467-8072-3e189275c35b:image.png)

**Checkpoint**

## Shell (If using lab computer)

See [Wikipedia](https://en.wikipedia.org/wiki/Shell_script) for some histories of the shell language. In short, this is the default language to interface with most UNIX terminal environments.

**Task 3.2.1 – Create a cleanup script.** Create a shell script at `<repo>/scripts/cleanup.sh` and change the permission to make it an executable. You also need to add the following shebang to the top of the script.

```bash
#!/usr/bin/sh
```

**Task 3.2.2 – Write the cleanup script.** This script will be used to quickly cleanup your workspace at the end of each section. Edit cleanup.sh to achieve the following functionalities:

1. Logout your GitHub account — you can add the following line to your shell script
    
    ```bash
    gh auth logout
    ```
    
2. Remove your workspace directory `~/autonomy_ws`.

**Checkpoint**

# Wrap Up

**Task 4.1 – Upload changes to GitHub.** Commit and push all changes to a new branch and create a PR and name it **Section 1**.

**Checkpoint**

**Task 4.2 – Cleanup (if on lab computer).** **Before doing this, make sure your changes are pushed to GitHub. Otherwise your work will be wiped without a backup!** 

Run `cleanup.sh` to clean up your workspace for the next section.

**Checkpoint**
