#ifndef A_STAR
#define A_STAR
#include "Eigen/Dense"
#include <iostream>
#include <map>
#include <vector>

/*
Header for my implimentation of the A* algorithm.  

*/

namespace star {

    class Node {
        private:
            bool obstical;
            
        public:

            Node(); // cordinates, h_cost, g_cost, parent
            Node(Eigen::Matrix<float,3,1>, float, float);
            Node(float, float, float, float, float);
            ~Node();

            Eigen::Matrix<float,3,1> cordinates;

            float x {};
            float y{};
            float z{};
            
            bool closedNode;

            void makeNodeObstical();
            void closeNode();

            float h_cost {};
            float g_cost {};
            float f_cost {};

            void get_f_cost();

    };

    class A_star {
        private:
            // const int num;
            const int obs_size{};
        public:

            A_star();
            A_star(Eigen::Matrix<float,3,1>, Eigen::Matrix<float,3,1>, Eigen::Matrix<float,3,Eigen::Dynamic>, float);
            ~A_star();

            int step;

            Eigen::Matrix<float,3,1> start_pose;
            Eigen::Matrix<float,3,1> end_pose;
            Eigen::Matrix<float,3,Eigen::Dynamic> obsticals;

            int shift_point(int);
            int m_to_mm(float);
            float mm_to_m(int);

            float distance_between(Eigen::Matrix<float,3,1>, Eigen::Matrix<float,3,1>);

            struct OpenNodesCostLogic { bool operator()(Node* const&,Node* const&);};

            struct ClosedNodesComparator { bool operator()(Node* const&, Node* const&);};
            
            std::map<Node*,Node*, OpenNodesCostLogic> openNodes;

            std::map<Node*,Node*, ClosedNodesComparator> closedNodes;

            std::map<Node*, Node*, ClosedNodesComparator> bufferNodes;

            void run();

            std::vector<Eigen::Matrix<float,1,3>>& look_around(const Eigen::Matrix<float,3,1>&, const int&, std::vector<Eigen::Matrix<float,1,3>>&);

            std::vector<Eigen::Matrix<float,1,3>> buffer;

            void generate_nodes(const std::vector<Eigen::Matrix<float,1,3>>&, Node* const&);
            
            std::vector<Eigen::Matrix<float,1,3>> final_path;

            bool is_already_closed(Node* const&);

            bool is_already_in_bufferNodes(Node* const&,Node* const&);

            std::vector<Eigen::Matrix<float,3,1>> obs;

    };
}



#endif // A_STAR