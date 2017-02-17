/***************************************************************************
 *  rosplan_interface_rcllrefbox.cpp - Referee box actions
 *
 *  Created: Wed Feb 16 22:37:18 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <ros/ros.h>

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <rcll_ros_msgs/ProductColor.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

class ROSPlanKbUpdaterMachineInfo {
 public:
	ROSPlanKbUpdaterMachineInfo(ros::NodeHandle &n)
		: n(n)
	{
		sub_machine_info_ = n.subscribe("rcll/machine_info", 10,
		                                &ROSPlanKbUpdaterMachineInfo::machine_info_cb, this);

		create_svc_update_knowledge();
		create_svc_current_knowledge();

		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "rs_predicate", cfg_rs_predicate_);
		GET_CONFIG(privn, n, "mps_type_predicate", cfg_mps_type_predicate_);
		GET_CONFIG(privn, n, "mps_state_predicate", cfg_mps_state_predicate_);
		GET_CONFIG(privn, n, "name_argument", cfg_name_argument_);
		GET_CONFIG(privn, n, "type_argument", cfg_type_argument_);
		GET_CONFIG(privn, n, "type_value_bs", cfg_type_value_bs_);
		GET_CONFIG(privn, n, "type_value_cs", cfg_type_value_cs_);
		GET_CONFIG(privn, n, "type_value_ds", cfg_type_value_ds_);
		GET_CONFIG(privn, n, "type_value_rs", cfg_type_value_rs_);
		GET_CONFIG(privn, n, "state_argument", cfg_state_argument_);
		GET_CONFIG(privn, n, "rs_ring1_argument", cfg_rs_ring1_argument_);
		GET_CONFIG(privn, n, "rs_ring2_argument", cfg_rs_ring2_argument_);
		GET_CONFIG(privn, n, "rs_ring_value_blue", cfg_rs_ring_value_blue_);
		GET_CONFIG(privn, n, "rs_ring_value_green", cfg_rs_ring_value_green_);
		GET_CONFIG(privn, n, "rs_ring_value_orange", cfg_rs_ring_value_orange_);
		GET_CONFIG(privn, n, "rs_ring_value_yellow", cfg_rs_ring_value_yellow_);
		GET_CONFIG(privn, n, "machine_instance_type", cfg_machine_instance_type_);

		relevant_predicates_ = {cfg_rs_predicate_, cfg_mps_type_predicate_, cfg_mps_state_predicate_};
		relevant_predicates_.sort();
		relevant_predicates_.unique();
		relevant_predicates_.remove_if([](const std::string &s) { return s.empty(); });

		rs_ring_colors_ = { {rcll_ros_msgs::ProductColor::RING_BLUE, cfg_rs_ring_value_blue_},
		                    {rcll_ros_msgs::ProductColor::RING_GREEN, cfg_rs_ring_value_green_},
		                    {rcll_ros_msgs::ProductColor::RING_ORANGE, cfg_rs_ring_value_orange_},
		                    {rcll_ros_msgs::ProductColor::RING_YELLOW, cfg_rs_ring_value_yellow_} };

		get_predicates();
	}

	void
	create_svc_update_knowledge()
	{
		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>
			("kcl_rosplan/update_knowledge_base_array", /* persistent */ true);

		ROS_INFO("Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void
	create_svc_current_knowledge()
	{
		svc_current_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>
			("kcl_rosplan/get_current_knowledge", /* persistent */ true);
		ROS_INFO("Waiting for ROSPlan service get_current_knowledge");
		svc_current_knowledge_.waitForExistence();
	}

	void
	create_svc_current_instances()
	{
		svc_current_instances_ =
			n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>
			("kcl_rosplan/get_current_instances", /* persistent */ true);
		ROS_INFO("Waiting for ROSPlan service get_current_instances");
		svc_current_instances_.waitForExistence();
	}

	void
	get_predicates()
	{
		// fetch and store predicate details
		ros::service::waitForService("kcl_rosplan/get_domain_predicate_details",ros::Duration(20));
		ros::ServiceClient pred_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>
			  ("kcl_rosplan/get_domain_predicate_details", /* persistent */ true);
		if (! pred_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("No service provider for get_domain_predicate_details");
			return;
		}

		for (const auto &pn : relevant_predicates_) {
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService pred_srv;
			pred_srv.request.name = pn;
			if(pred_client.call(pred_srv)) {
				std::string pred_str;
				std::for_each(pred_srv.response.predicate.typed_parameters.begin(),
				              pred_srv.response.predicate.typed_parameters.end(),
				              [&pred_str](const auto &kv) { pred_str += " " + kv.key + ":" + kv.value; });
				ROS_INFO("Relevant predicate: (%s%s)", pn.c_str(), pred_str.c_str());
				predicates_[pn] = pred_srv.response.predicate;
			} else {
				ROS_ERROR("Failed to get predicate details for %s", pn.c_str());
				return;
			}
		}
	}

	void
	check_predicate(const std::string &predicate_name,
	                const std::string &idvar_name, const std::string &idvar_value,
	                const std::map<std::string, std::string> &expected_values,
	                rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &remsrv,
	                rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &addsrv)
	{
		if (predicates_.find(predicate_name) != predicates_.end()) {
			rosplan_knowledge_msgs::GetAttributeService srv;
			srv.request.predicate_name = predicate_name;
			if (! svc_current_knowledge_.isValid()) {
				create_svc_current_knowledge();
			}
			if (svc_current_knowledge_.call(srv)) {
				bool not_found_at_all = true;
				for (const auto &a : srv.response.attributes) {
					std::map<std::string, std::string> arguments;
					for (const auto &kv : a.values)  arguments[kv.key] = kv.value;

					if (arguments.find(idvar_name) != arguments.end() &&
					    arguments[idvar_name] == idvar_value)
					{
						not_found_at_all = false;
						if (std::any_of(expected_values.begin(), expected_values.end(),
						                [&arguments](const auto &ev) -> bool
						                { return (arguments.find(ev.first) != arguments.end() &&
						                          arguments[ev.first] != ev.second); }))
							
						{
							ROS_INFO("Updating '%s' for '%s'", predicate_name.c_str(), idvar_value.c_str());
							rosplan_knowledge_msgs::KnowledgeItem new_a = a;
							std::for_each(expected_values.begin(), expected_values.end(),
							              [&new_a](auto &ev) {
								              for (auto &kv : new_a.values) {
									              if (kv.key == ev.first) {
										              kv.value = ev.second;
										              break;
									              }
								              }
							              });

							remsrv.request.knowledge.push_back(a);
							addsrv.request.knowledge.push_back(new_a);
						}
						// we do NOT break here, by this, we also remove any additional
						// fact that matches the idvar and ensure data uniqueness.
					}
				}
				if (not_found_at_all) {
					// It does not exist at all, create
					rosplan_knowledge_msgs::KnowledgeItem new_a;
					new_a.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					new_a.attribute_name = predicate_name;
					{ // Add ID argument
						diagnostic_msgs::KeyValue kv;
						kv.key = idvar_name; kv.value = idvar_value;
						new_a.values.push_back(kv);
					}
					std::for_each(expected_values.begin(), expected_values.end(),
					              [&new_a](const auto &ev) {
						              diagnostic_msgs::KeyValue kv;
						              kv.key = ev.first; kv.value = ev.second;
						              new_a.values.push_back(kv);
					              });
					addsrv.request.knowledge.push_back(new_a);
					ROS_INFO("Adding '%s' info for %s", predicate_name.c_str(), idvar_value.c_str());
				}
			} else {
				ROS_ERROR("Failed to call '%s' for '%s'",
				          svc_current_knowledge_.getService().c_str(), predicate_name.c_str());
			}
		}
	}
	
	void
	send_machine_predicate_updates(const rcll_ros_msgs::Machine &m)
	{
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray remsrv;
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;

		remsrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::REMOVE_KNOWLEDGE;
		addsrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE;

		check_predicate(cfg_mps_type_predicate_, cfg_name_argument_, m.name,
		                { {cfg_type_argument_, m.type} },
		                remsrv, addsrv);

		check_predicate(cfg_mps_state_predicate_, cfg_name_argument_, m.name,
		                { {cfg_state_argument_, m.state} },
		                remsrv, addsrv);

		if (m.type == "RS") {
			if (m.rs_ring_colors.size() == 2) {
				// Info has actually been provided
				check_predicate(cfg_rs_predicate_, cfg_name_argument_, m.name,
				                { {cfg_rs_ring1_argument_, rs_ring_colors_[m.rs_ring_colors[0]]},
					                {cfg_rs_ring2_argument_, rs_ring_colors_[m.rs_ring_colors[1]]} },
				                remsrv, addsrv);
			}
		}

		if (! remsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(remsrv)) {
				ROS_ERROR("Failed to remove predicates");
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("Failed to add predicates");
				return;
			}
		}
	}

	void
	send_machine_instance_updates(const std::vector<rcll_ros_msgs::Machine> &machines)
	{
		rosplan_knowledge_msgs::GetInstanceService srv;
		srv.request.type_name = cfg_machine_instance_type_;
		if (! svc_current_instances_.isValid()) {
			create_svc_current_instances();
		}
		if (! svc_current_instances_.call(srv)) {
			ROS_ERROR("Failed to retrieve current instances of type '%s'",
			          cfg_machine_instance_type_.c_str());
			return;
		}

		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;

		std::vector<std::string> instances = srv.response.instances;
		std::sort(instances.begin(), instances.end());
		for (const rcll_ros_msgs::Machine &m : machines) {
			if (! std::binary_search(instances.begin(), instances.end(), m.name)) {
				rosplan_knowledge_msgs::KnowledgeItem new_i;
				new_i.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				new_i.instance_type = cfg_machine_instance_type_;
				new_i.instance_name = m.name;
				addsrv.request.knowledge.push_back(new_i);
				ROS_INFO("Adding missing instance '%s - %s'",
				         new_i.instance_name.c_str(), new_i.instance_type.c_str());
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("Failed to add machine instances");
				return;
			}
		}
	}

	void
	machine_info_cb(const rcll_ros_msgs::MachineInfo::ConstPtr& msg)
	{
		//ROS_INFO("Sending updates (conditionally)");
		send_machine_instance_updates(msg->machines);
		for (const rcll_ros_msgs::Machine &m : msg->machines) {
			send_machine_predicate_updates(m);
		}
		last_machine_info_ = msg;
	}

 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_machine_info_;
	ros::ServiceClient svc_update_knowledge_;
	ros::ServiceClient svc_current_knowledge_;
	ros::ServiceClient svc_current_instances_;

	std::string cfg_bs_predicate_;
	std::string cfg_cs_predicate_;
	std::string cfg_ds_predicate_;
	std::string cfg_rs_predicate_;
	std::string cfg_mps_type_predicate_;
	std::string cfg_mps_state_predicate_;
	std::string cfg_name_argument_;
	std::string cfg_type_argument_;
	std::string cfg_type_value_bs_;
	std::string cfg_type_value_cs_;
	std::string cfg_type_value_ds_;
	std::string cfg_type_value_rs_;
	std::string cfg_state_argument_;
	std::string cfg_rs_ring1_argument_;
	std::string cfg_rs_ring2_argument_;

	std::string cfg_rs_ring_value_blue_;
	std::string cfg_rs_ring_value_green_;
	std::string cfg_rs_ring_value_orange_;
	std::string cfg_rs_ring_value_yellow_;

	std::string cfg_machine_instance_type_;

	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates_;
	std::list<std::string> relevant_predicates_;

	std::map<int, std::string> rs_ring_colors_;
	
	rcll_ros_msgs::MachineInfo::ConstPtr last_machine_info_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_interface_behaviorengine");

	ros::NodeHandle n;

	ROSPlanKbUpdaterMachineInfo rosplan_kb_updater(n);
	
  ros::spin();
  
	return 0;
}
