#include "A_star.h"
using namespace std;

void star::Node::get_f_cost() {
   std::cout << f_cost << endl;
}

star::Node::~Node() {
    delete this;
}

star::Node::Node(Eigen::Matrix<float,3,1> _cordinates, float _h_cost, float _g_cost) : cordinates {_cordinates}, h_cost {_h_cost}, g_cost {_g_cost}{
    f_cost = h_cost+g_cost;
    obstical = false;
    closedNode = false;
};

star::Node::Node(float x, float y, float z, float _h_cost, float _g_cost) : h_cost{_h_cost}, g_cost{g_cost}, x{x}, y{y}, z{z}{
    cordinates(0) = x;
    cordinates(1) = y;
    cordinates(2) = z;
    f_cost = h_cost + g_cost;
    obstical = false;
    obstical = false;
}

star::Node::Node() {};

void star::Node::closeNode() {
    closedNode = true;
}

void star::Node::makeNodeObstical() {
    obstical = true;
}


star::A_star::~A_star() {
    // std::cout << "A_star Destructed" << std::endl;
}


star::A_star::A_star() {};

bool star::A_star::OpenNodesCostLogic::operator()(Node* const& a, Node* const& b) {
    // Checks for equality between coordinates.  Used to see if nodes has already been created at "these" coordinates
    return (std::tie(a->f_cost, a->h_cost, a->cordinates(0), a->cordinates(1), a->cordinates(2)) < std::tie(b->f_cost, b->h_cost, b->cordinates(0), b->cordinates(1), b->cordinates(2))); 
    // Responsible for:
    // 1) ordering nodes in decending order.
    // 2) still adding nodes of same f and h cost even if they are at different coordinates
    // 3) Not adding a node that has been opened with the same coordinates and g cost
    // 4) Adding nodes of the same coordinates but of different g cost
    // --> For nodes that get added due to case 4) with a larger g cost.  The cosedNodes comparitor will take care of it.
} 

bool star::A_star::ClosedNodesComparator::operator()(Node* const& a, Node* const& b) {
    return std::tie(a->cordinates(0), a->cordinates(1), a->cordinates(2)) < std::tie(b->cordinates(0), b->cordinates(1), b->cordinates(2));
    // Responsible for:
    // 1) Not closing a node at the same coordinates twice
    // 2) Not closing a node the was added to openNodes with a higher g cost.  Special sub case of 1)
}

// Sequence of comparison for opening a node:
// 1) open a node
// 2) Check if it is in closedNodes
// --> if it is do nothin and open another node, if not:
// 3) add if to opened nodes

// Sequence of comparison for closing a node:
// 1) Remove begin() node from openNode
// 2) Check if it is in closedNodes
// --> if it is do nothing and continue to closing next node, if not:
// 3) add to closedNodes


star::A_star::A_star(Eigen::Matrix<float,3,1> _start_pose, Eigen::Matrix<float, 3, 1> _end_pose, Eigen::Matrix<float,3,Eigen::Dynamic> _obsticals, float _step) : obs_size(_obsticals.cols()){
    int n = 3;
    obsticals.resize(n,_obsticals.cols());

    step = m_to_mm(_step); // UNITS: mm
    
    for (int i=0; i<3; i++) {
        start_pose(i) = m_to_mm(_start_pose(i)); // UNITS: mm
        end_pose(i) = m_to_mm(_end_pose(i)); // UNITS: mm

        start_pose(i) = shift_point(start_pose(i));
        end_pose(i) = shift_point(end_pose(i));
    }

    for (int i=0; i<_obsticals.cols(); i++) {
        for (int j=0; j<3; j++) {
            obsticals(j,i) = m_to_mm(_obsticals(j,i)); // UNITS: mm
            obsticals(j,i) = shift_point(obsticals(j,i));
            // cout << obsticals(j,i) << endl;
        }
       
        openNodes[new star::Node(obsticals.col(i).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())] = new star::Node(obsticals.col(i).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());;
        bufferNodes[new star::Node(obsticals.col(i), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())] = new star::Node(obsticals.col(i), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());;
        obs.push_back(obsticals.col(i));
        auto explored_cords = look_around(obsticals.col(i),step,buffer);
        for(auto itr = explored_cords.begin(); itr!=explored_cords.end();itr++) {
            openNodes[new star::Node((*itr).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())] = new star::Node((*itr).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());;
            bufferNodes[new star::Node((*itr).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())] = new star::Node((*itr).transpose(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());;
            obs.push_back((*itr));

        }

    }


    star::Node *startNode = new star::Node(start_pose, distance_between(start_pose,end_pose), 0);
    openNodes[startNode], bufferNodes[startNode] = startNode;

    star::A_star::run();

    // for( auto tt = obs.begin(); tt != obs.end(); tt++) {
    //     cout << (*tt).transpose() << endl;
    // }
};


void star::A_star::run() {
    while (true) {
        Node* current_node = openNodes.begin()->first; 

        if (current_node->h_cost < step) {
            closedNodes[current_node] = openNodes[current_node]; // add current node to closed_nodes

            while (true) {
                final_path.push_back(current_node->cordinates);

                if (current_node->cordinates == start_pose) {

                    break;
                }
                else {
                    current_node = closedNodes[current_node];
                }
            }

            break;

        }
        else {
            closedNodes[current_node] = openNodes[current_node]; // add current node to closed_nodes
            star::A_star::generate_nodes(star::A_star::look_around(current_node->cordinates,step, buffer), current_node);
        }

        openNodes.erase(current_node);
    }

    for (auto i = final_path.begin(); i != final_path.end(); i++) {
        cout << (*i) << endl;
    }

}

bool star::A_star::is_already_closed(Node* const& node) {
    return (closedNodes.find(node) != closedNodes.end());
}

bool star::A_star::is_already_in_bufferNodes(Node* const& node, Node* const& currentNode) {
    auto itr = bufferNodes.insert(std::pair<Node*,Node*>(node,currentNode)); // check if in buffer
    if (itr.second == false) { // node was not added (already in bufferNodes). check g cost
        if (node->g_cost < itr.first->first->g_cost) { // smaller g cost than bufferNodes
            // cout << "found lower g_cost\n";
        
            openNodes[itr.first->first]->g_cost = node->g_cost; // update openNodes g cost at this node.  DOES NOT UPDATE THE ORDERING OF OPENNODES... assuming step size is small enough to not notice effect.
            itr.first->first->g_cost = node->g_cost; // update g cost in bufferNodes

        }
        return true; // No nodes or values need to be added or updated. 
    }
    return false; // Node was added, coordinates were not found. openNodes needs to be updated
}

void star::A_star::generate_nodes(const std::vector<Eigen::Matrix<float,1,3>>& cords, Node* const& current_node) {

    for (int z=0;z<cords.size();z++) {

        Node* test_node = new star::Node(cords[z],distance_between(cords[z],end_pose),current_node->g_cost + distance_between(cords[z], current_node->cordinates));

        if (is_already_closed(test_node)) {
            continue;
        }
        if (is_already_in_bufferNodes(test_node, current_node)) {
            continue;
        }
        openNodes[test_node] = current_node;
    }
}

int star::A_star::m_to_mm(float input) {
    return static_cast<int>(input*1000);
}

float star::A_star::mm_to_m(int input) {
    return static_cast<float>(input/1000);
}

float star::A_star::distance_between(Eigen::Matrix<float,3,1> start, Eigen::Matrix<float,3,1> end) {
    return sqrt(pow(end(0)-start(0),2) + pow(end(1)-start(1),2) + pow(end(2)-start(2),2)); // absolute is implicit
}


int star::A_star::shift_point(int cordinate) {
    int remainder = cordinate % step;

    if (cordinate != abs(cordinate)) {
        remainder = step + remainder;
    }

    if (remainder == 0) {
        return cordinate;
    }
    else {
        if ((step - remainder) > remainder) {
            cordinate = cordinate - remainder;
        }
        else {
            cordinate = cordinate + (step-remainder);
        }
    }
    return cordinate;
}


std::vector<Eigen::Matrix<float,1,3>>& star::A_star::look_around(const Eigen::Matrix<float,3,1>& cordinate, const int& step, std::vector<Eigen::Matrix<float,1,3>>& buffer) {

    Eigen::Matrix<float,1,3> buffer_row;

    int x;
    int y;
    int z;
    for (int i= 0; i < 3; i++) {
        x = i;
        if (i == 2) {
            x = -1;
        }
        for (int j = 0; j < 3; j++) {
            y = j;
            if (j == 2) {
                y = -1;
            }
            for (int k = 0; k < 3; k++) {
                z = k;
                if (k == 2) {
                    z = -1;
                }

                buffer_row(0) = cordinate(0)+step*x;
                buffer_row(1) = cordinate(1)+step*y;
                buffer_row(2) = cordinate(2)+step*z;


                buffer.push_back(buffer_row);
            }            
        }
    }
    
    buffer.erase(buffer.begin()); // erase the node off of list that we are searching around


    return buffer;
}
