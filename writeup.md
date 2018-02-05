## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[dhimage]: ./misc_images/dh.jpg
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/image-4.png
[image5]: ./misc_images/l21-l-inverse-kinematics-new-design-fixed.png
[image6]: ./misc_images/image-5.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.  

Here is a diagram of Kuka KR210 robot link assignment.

![alt text][dhimage]

Here is a DH parameter table derived from `kr210.urdf.xacro` file.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

First, I've defined method for creating modified DH transformation matrix with alpha, a, d, q values.
```python
def TF_Matrix(alpha, a, d, q):
  TF = Matrix([[            cos(q),           -sin(q),           0,             a],
               [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
               [                 0,                 0,           0,             1]])
  return TF
```

Then, I created individual matrices using this method.
```python
T0_1  = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2  = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3  = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4  = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5  = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6  = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

Final homogeneous transform can be generated like this.
```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

**Step 1.** Find position of WC relative to the base frame. It can be obtained by this equation.
![alt text][image4]
The implementation is as below.
```python
# Define RPY rotation matrices
ROT_x = Matrix([[       1,       0,       0],
                [       0,  cos(r), -sin(r)],
                [       0,  sin(r),  cos(r)]])

ROT_y = Matrix([[  cos(p),       0,  sin(p)],
                [       0,       1,       0],
                [ -sin(p),       0,  cos(p)]])

ROT_z = Matrix([[  cos(y), -sin(y),       0],
                [  sin(y),  cos(y),       0],
                [       0,       0,       1]])

ROT_EE = ROT_z * ROT_y * ROT_x

# More information can be found in KR210 Forward Kinematics section
ROT_Error = ROT_z.subs(y, pi) * ROT_y.subs(p, -pi/2.)
ROT_EE = ROT_EE * ROT_Error
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

EE = Matrix([[px],
             [py],
             [pz]])

WC = EE - (0.303) * ROT_EE[:,2]
```

**Step 2.** Find theta1, theta2, theta3. theta2 and theta3 can be calculated according to this diagram.
![alt text][image5]
The implementation is as below.
```python
theta1 = atan2(WC[1], WC[0])

# SSS triangle for theta2 and theta3
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m
```

**Step 3.** Find R3_6 according to this equation.
![alt text][image6]
The implementation is as below.
```python
R3_6 = Transpose(R0_3) * ROT_E
```

**Step 4.** Find theta4, theta5, theta6 with `R3_6`
```python
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

- **Line 30 to 37** Create symbols to be used in DH tranformation matrices
- **Line 40 to 46** Create modified DH parameters
- **Line 49 to 54** Define method that returns DH Transformation matrix by parameters
- **Line 57 to 63** Create individual matrices
- **Line 65** Calculate T0_EE transformation Matrix
- **Line 71 to 85** Define rotation Matrix
- **Line 88 to 89** Calculate rotation error
- **Line 101 to 107** Extract provided EE position and rotation
- **Line 112 to 117** Calculate WC Position
- **Line 120 to 144** Calculate theta1 to theta6

At first, I have used `.inv('LU')` method to get `R3_6` in line 138. But the result made robot movement too complicated. I have changed it to simply `Transpose()`, which the result is identical in mathematics, then the movement got a lot smoother.

My simulator shows the robot is very accurately following paths. But it fails to grab bottles. And it also happens in demo mode.

Defining symbols and matrices outside of `handle_calculate_IK(req)` could speed up execution time.
