/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CNODEREGISTRATIONDECIDER_CM_IMPL_H
#define CNODEREGISTRATIONDECIDER_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CNodeRegistrationDecider_CM<GRAPH_T>::CNodeRegistrationDecider_CM() {}

template<class GRAPH_T>
CNodeRegistrationDecider_CM<GRAPH_T>::~CNodeRegistrationDecider_CM() {}

template<class GRAPH_T>
void CNodeRegistrationDecider_CM<GRAPH_T>::addNodeAnnotsToPose(
		global_pose_t* pose) const { }

template<>
void CNodeRegistrationDecider_CM<mrpt::graphs::CNetworkOfPoses2DInf_NA>::addNodeAnnotsToPose(
		global_pose_t* pose) const {
	ASSERT_(pose);

	pose->agent_ID_str = this->own_ns;
	// ASSUMPTION: addNodeAnnotsToPose is going to be called right before the
	// actual registration.
	// Mark it with the nodeID that is up-next 
	pose->nodeID_loc = this->m_graph->nodeCount();
}

} } } // end of namespaces

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_CM_IMPL_H */
