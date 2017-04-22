# TODO CLI: inputs: skeleton CSV from, skeleton CSV onto, 'b' or 'e' trust start flag, destination_CSV name, transform_dest name

# TODO read CSV from
# TODO read CSV onto

# TODO select canonical skeletons of associated times

# TODO find mapping using:
# /** \brief Estimate a rigid rotation transformation between a source and a target point cloud.
#   101           * \param[in] cloud_src the source point cloud dataset
#   102           * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
#   103           * \param[in] cloud_tgt the target point cloud dataset
#   104           * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
#   105           * \param[out] transformation_matrix the resultant transformation matrix
#   106           */
#   107         virtual void
#   108         estimateRigidTransformation (
#   109             const pcl::PointCloud<PointSource> &cloud_src,
#   110             const std::vector<int> &indices_src,
#   111             const pcl::PointCloud<PointTarget> &cloud_tgt,
#   112             const std::vector<int> &indices_tgt,
#   113             Matrix4 &transformation_matrix) const;



# TODO transform all of from into onto as from_onto

# TODO output from_onto to CSV