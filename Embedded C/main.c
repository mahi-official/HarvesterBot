/*
* Fuctions: 				set_adjcent_nodes, source_to_destination, pre_pluck, post_pluck, send_signal
* Global Variables:	size, trees, adj
*/
//**********************************************************************************************
//**********************************************************************************************
//for using "port.c" functions & global variables
#include "port.c"
//**********************************************************************************************
//**********************************************************************************************
//Defining Number of Trees
#define size 2
//Trees Node Number Entry
int trees[] = {16, 32};
//Adjcent nodes to trees
int adj[size * 4][2];
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			set_adjcent_nodes
* Input:     						None
* Output:    					None 
* Logic:     						Finding & Storing Adjcent Nodes to the Trees
*										Dividing tree node by 7 & using quotient & remainder for finding & storing adj Nodes in a 2D array
*										Also storing the direction of bot to be Moved to Check Tree in 2nd Column
* Example Call:   			set_adjcent_nodes();
*/
void set_adjcent_nodes() 
{
	//Variables used as counters
    int x=0,i=0,j=0;
	//Stores the value of tree node as x,y coordinates
	int tree_node[size][2];
	
    while (i < size && x < (size * 4))
	{
		tree_node[i][0] = trees[i] / 7;
		tree_node[i][1] = trees[i] % 7;
		
		if(tree_node[i][0] < 0 || tree_node[i][1] < 0 || tree_node[i][0] > 6 || tree_node[i][1] > 6)
		{
            tree_node[i][0] = -1;
            tree_node[i][1] = -1;
		    
			for(j = 0; j < 4; j++)
			{
                adj[x + j][0] = -1;
                adj[x + j][1] = 0;
		    }
			i++;
    	    x+=4;
            continue;
        }
    	
		for(j = 0; j < 4; j++)
		{
            int nodeX [2];
            int face;
            
			switch(j)
			{
                case 0: nodeX[0] = tree_node[i][0] - 1;
                        nodeX[1] = tree_node[i][1];
                        face = 0;
                        break;
                case 1: nodeX[0] = tree_node[i][0];
                        nodeX[1] = tree_node[i][1] - 1;
                        face = 90;
                        break;
                case 2: nodeX[0] = tree_node[i][0] + 1;
                        nodeX[1] = tree_node[i][1];
                        face = 180;
                        break;
                case 3: nodeX[0] = tree_node[i][0];
                        nodeX[1] = tree_node[i][1] + 1;
                        face = 270;
                        break;
                default:nodeX[0] = -1;
                        nodeX[1] = -1;
                        face = 0;
                        break;
            }
            
			adj[x + j][0] = (nodeX[0] * 7) + nodeX[1];
            adj[x + j][1] = face;
            
			if(nodeX[0] < 0 || nodeX[1] < 0 || nodeX[0] > 6 || nodeX[1] > 6)
			{
                adj[x + j][0] = -1;
                adj[x + j][1] = 0;
            }
        }

		i++;
    	x+=4;
    }
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			source_to_destination
* Input:     						dNode: Enters the Destination Node for the bot to reach
*										mode: States Which is the preffered mode for Travesing the Bot (Row Mode/ Column Mode)
*										rotation: Rotation of rotation of bot toward the tree (after reaching the node)
* Output:    					None 
* Logic:     						Source to Destination Function For Traversing Firebird to particular Node
*										Depending upon the mode of movement the bot moves via Row or column mode for better processing
* Example Call:   			source_to_destination(20, 1, 0);
*/
void source_to_destination(int dNode, int mode, int rotation)
{
	//initial();
    if (mode == 1)
    {
	    while(1)
        {
            while(1)
                {
				if (move_row(dNode) == 1)
                    break;
				}

            if (move_col(dNode) == 1)
                break;
        }
    }
	else
        while(1)
        {
            while(1)
                {
				if (move_col(dNode) == 1)
                    break;
				}
            if (move_row(dNode) == 1)
                break;
        }

    rotate_to_position(rotation);           //FireBird will face to that position
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			post_pluck
* Input:     						None
* Output:    					None 
* Logic:     						Runs the Post Plucking Process of the Firebird
*										moves back & stops
s* Example Call:   			post_pluck();
*/
void post_pluck()
{
	back();
	velocity(200,200);
	_delay_ms(250);
	stop();
}
//**********************************************************************************************
//**********************************************************************************************
void traverse_adj_nodes()
{
	set_adjcent_nodes();
	for(int i= 0; i<size*4;i++)
	{
		a=tree_node[i];
		source_to_destination(a,0,90);
	}
}
//**********************************************************************************************
//**********************************************************************************************
/*
* Function Name:			send_signal
* Input:     						None
* Output:    					None 
* Logic:     						Sends a signal from Firebird to Rasberry Pi to start Image Processing
*										It uses GPIO PIN
* Example Call:   			send_signal();
*/
 void send_signal()
 {
	 blink();
 }
//**********************************************************************************************
//**********************************************************************************************
/*
* Main:							send_signal
* Logic:     						It runs all the functins Required for Runing Firebird
*/
int main(void)
{
	init_devices();
	_delay_ms(1000);
	traverse_adj_nodes();
	}
}