# Geometry Notation Standard

This document serves as a brainstorming page for our notation standard.  Once the ideas have been fleshed out, we will draft up a more formal page on notation, which will include an associated figure depicting co-ordinate frame and short-hand frame tags.  Free free to add notes, expand thoughts, and add to the following sub-topics.

## Notation Goals
* mitigate ambiguity regarding transformation and rotation "from-to" frames and direction
* mitigate ambiguity regarding vector quantities (position, linear velocity, angular velocity, etc), with respect to the vector head-tail relationships, and what frame the vector is expressed in.

## Usage of Pre-scripts?
* pre-scripts are awkward to some, though allow for "cancellation" of frames when performing multiplication, for example

```
B_wx_B_C //angular velocity of frame C, wrt B, expressed in frame B
Rx_A_B // rotation from frame B to A
A_wx_B_C //angular velocity of frame C, wrt B, expressed in frame A

A_wx_B_C = Rx_A_B * B_wx_B_C

```
In the above example, the B frame "cancels" when writing out the expression for A_wx_B_C.  Also, the use of Rx_A_B, instead of R_A_B, will make sure that Rx is denoted as the rotation, instead of just R, which may be confused for some frame that's also named R. 

* It is hard to distinguish between variables and frames, especially if either frames or variables can be first.  Is W_p  $W_p$ or ${}_Wp$? Variables could all be two or more characters, while frames are one.  So $omega$ is rotation rate, $Rot$ is rotation matrix, $Tran$ is a transformation.  Not sure about how readable this is. (SLW)

## Length of Tags
* 1 letter tags, we risk running out of "intuitive" letters for frames, however, if we maintain a central list of co-ordinate frames and their associated tags, this doesn't seem like a big deal. 
    * I think it is possible to maintain a consistent list. Just having the list will be so valuable. (SLW)

## Templating Types Based on Frames
* Leo to fill in more detail, including an example.


## Example use cases

### Change of coordinate frame for a point seen in a camera
- C = Camera frame
- B = Body frame
- M = map frame
- Tran = Transformation
- point = point

$ {}_Mp = {}_MT_{MB}{}_BT_{BC}{}_Cp $

M_point = M_Tran_M_B * B_Tran_B_C * C_point


### Gimballed camera measurement

To be added

### Quadrotor dynamics

To be added


