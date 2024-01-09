# TODO:

- Very important: I need to keep track of when the insole socket was connected and report this time as the correct time for synchronization. 

- I also want to keep track of how much delay was accrued during the measurement!!!! This is key because ID wont the be able to synchronize anymore if this gets too big. 

	Basically what I need to do is convert the insole time field to estimate what is the current measurement's time and compare it with ros::Time::now() (rospy.Time.now() probably in python, or something like that) and if is more than the delay buffer we added in id (140 ms + 100ms), that has to get the TFs from the IK (can take up to 15ms sometimes, everything else seems pretty fast so they dont matter), then the ID async will no longer have enough time to synchronize the measurements. A better idea is to do what H. asked for and have the "completeness" of insole measurements be the trigger for ID. This is hard to implement in practice because I will need to have the wrenchbuffer keep track of what has been published already and invert the buffer idea, as in, get all the IKs, keep it in a list and then every 100ms or something see what is the newest complete wrench buffer over all wrenches that we are keeping track of. For this we need some sort of wrench manager to keep track of all the forces and give some sort of timestamp of what is available, which is doable, but more complicated. 

## Old todo, not sure if it is relevant anymore

IDK about this, i think i matured this idea for longer now, it seems okay, but now we know we need a synchronization event, as in, the first measurement i get from the insole already has some unmeasurable delay added to it

### (OLD)

The most rossy way to deal with the delay, and it is not very ROSsy, but I think it is the best possible scenario where we don't need to deal with so many moving parts is to restructure this node as follows:

We will measure somehow the delay between the force production and force measurement. Let's approximate first to about 100ms. 

Everything will be timestamped with this older delay, so now we have the actual estimated time at which things really happened.

Then we will change the way that ID works. To keep the whole 100hz thing, we will instead use a common wrench publishing interface and read from that interface each time we get an IK message.

So the new ID version of the node will read from a single input and get the wrenches (can be more than one) from this interface. 

