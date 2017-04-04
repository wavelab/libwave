# Geometry Notation Stanard

This document serves as a brainstorming page for our notation standard.  Once the ideas have been fleshed out, we will draft up a more formal page on notation, which will include an associated figure depecting co-ordinate frame and short-hand frame tags.  Free free to add notes, exapand thoughts, and add to the following sub-topics.

## Notation Goals
* mitigate ambiguity regarding transfomration and rotation "from-to" frames and direction
* mitigate ambiguity regarding vector quantities (position, linear velocity, angular velocity, etc), with respect to the vector head-tail relationships, and what frame the vector is expressed in.

## Usage of Pre-scripts?
* presectipts are akward to some, though allow for "cancellation" of frames when performing multiplication, for example

```
B_wx_B_C //angular velocity of frame C, wrt B, expressed in frame B
Rx_A_B // rotation from frame B to A
A_wx_B_C //angular velocity of frame C, wrt B, expressed in frame A

A_wx_B_C = Rx_A_B * B_wx_B_C

```
In the above example, the B frame "cancels" when writing out the expression for A_wx_B_C.  Also, the use of Rx_A_B, instead of R_A_B, will make sure that Rx is denoted as the rotation, instead of just R, which may be confused for some frame that's also named R. 

## Length of Tags
* 1 letter tags, we risk running out of "intuative" letters for frames, however, if we mainitain a central list of co-ordinate frames and their associated tags, this doesn't seem like a big deal. 

## Templating Types Based on Frames
* Leo to fill in more detail
