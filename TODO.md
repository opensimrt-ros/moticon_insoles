# TODO:

The most rossy way to deal with the delay, and it is not very ROSsy, but I think it is the best possible scenario where we don't need to deal with so many moving parts is to restructure this node as follows:

We will measure somehow the delay between the force production and force measurement. Let's approximate first to about 100ms. 

Everything will be timestamped with this older delay, so now we have the actual estimated time at which things really happened.

Then we will change the way that ID works. To keep the whole 100hz thing, we will instead use a common wrench publishing interface and read from that interface each time we get an IK message.

So the new ID version of the node will read from a single input and get the wrenches (can be more than one) from this interface. 
