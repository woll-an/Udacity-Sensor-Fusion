// Structure to represent node of kd tree
template <typename PointT>
struct Node {
  PointT point;
  int id;
  Node* left;
  Node* right;

  Node(PointT point, int setId)
      : point(point), id(setId), left(NULL), right(NULL) {}
};

template <typename PointT>
struct KdTree {
  Node<PointT>* root;

  KdTree() : root(NULL) {}

  void insert(PointT point, int id) {
    Node<PointT>* newNode = new Node<PointT>(point, id);
    if (!root) {
      root = newNode;
    } else {
      insertNode(newNode, root, 0);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(PointT target, float distanceTol) {
    std::vector<int> ids;
    searchNode(target, distanceTol, root, ids, 0);
    return ids;
  }

 private:
  void searchNode(PointT& target,
                  float distanceTol,
                  Node<PointT>* node,
                  std::vector<int>& ids,
                  int depth) {
    if (!node) {
      return;
    }

    float distX = node->point.x - target.x;
    float distY = node->point.y - target.y;
    float distZ = node->point.z - target.z;

    bool inBox = std::abs(distX) <= distanceTol &&
                 std::abs(distY) <= distanceTol &&
                 std::abs(distZ) <= distanceTol;

    if (inBox) {
      ids.emplace_back(node->id);
      searchNode(target, distanceTol, node->left, ids, depth + 1);
      searchNode(target, distanceTol, node->right, ids, depth + 1);
    } else {
      float dist = 0;
      switch (depth % 3) {
        case 0:
          dist = distX;
          break;
        case 1:
          dist = distY;
          break;
        case 2:
          dist = distZ;
          break;
      }
      if (dist > 0) {
        searchNode(target, distanceTol, node->left, ids, depth + 1);
      } else {
        searchNode(target, distanceTol, node->right, ids, depth + 1);
      }
    }
  }
  void insertNode(Node<PointT>* newNode, Node<PointT>* currentNode, int depth) {
    bool goRight = false;
    switch (depth % 3) {
      case 0:
        goRight = newNode->point.x > currentNode->point.x;
        break;
      case 1:
        goRight = newNode->point.y > currentNode->point.y;
        break;
      case 2:
        goRight = newNode->point.z > currentNode->point.z;
        break;
    }
    Node<PointT>* child;
    if (goRight) {
      if (currentNode->right) {
        insertNode(newNode, currentNode->right, ++depth);
      } else {
        currentNode->right = newNode;
      }
    } else {
      if (currentNode->left) {
        insertNode(newNode, currentNode->left, ++depth);
      } else {
        currentNode->left = newNode;
      }
    }
  }
};
