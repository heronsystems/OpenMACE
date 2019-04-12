#ifndef DOXY_MAINPAGE_H
#define DOXY_MAINPAGE_H

/** \mainpage MACE class reference manual
 * \section intro Introduction
 *
 * The Multi-Agent Cooperative Engagement (MACE) framework is a framework linking the communications, control, automation and human-machine interface components of a practical multi-vehicle system into a deployable package. MACE establishes the data management, scheduling, and monitoring required by a multi-vehicle robotic system.  Designed as a modular software architecture, MACE implements a core collaborative engine that exposes interfaces with other system components via APIs. This approach abstracts the details of the collaboration away from an individual system, allowing for rapid integration of third party components.
 *
 * \htmlonly <style>div.image img[src="maceArchitecture.png"]{width:700px;}</style> \endhtmlonly
 * \image html maceArchitecture.png width=2cm
 * \image latex maceArchitecture.eps "MACE Architecture" width=10cm 
 * 
 * The modular architecture of MACE allows the software to be agnostic to the inner workings of a vehicle and its motion primitives. MACE implements a common interface between vehicle communications and the core software, and exposes a flexible API for vehicle developers to interface with other MACE enabled vehicles. Fundamentally, MACE is the backbone communications architecture that can facilitate a Swarm architecture (i.e. vehicle-to-vehicle communications, both 1-to-1 and 1-to-many). MACE provides a simple user interface, however MACE also implements a modular ground station API. Similar to the vehicle communications, this provides methods for third party developers to implement their own human-machine interfaces. Finally, MACE provides a resource and task allocation (RTA) API. This abstracts the tasking of a single vehicle within a swarm or the entire collective swarm to simple waypoints and commands. MACE can be applied to air vehicles, ground vehicles, surface vehicles, or a mixture of different vehicle types. The MACE architecture has been under active development since 2015 and is currently maturing alongside path planning and RTA research programs.
 * 
 * 
 * \section usage Usage
 * MACE can be deployed both on the ground on any Windows or Linux operating system as well as in the air. Currently, testing has been conducted for air instances deployed on ODROID XU4 single board computers with Ubuntu Mate, an Ubuntu derivative, operating system installed. The ODROID was used for its lightweight and compact form factor, which allows us to mount it on most COTS UAS. This illustrates a key benefit of MACE -- the ability to rapidly retrofit cost effective vehicles to gain intelligent cooperative capabilities. 
 * 
 * Integrating with MACE is best explained using an example. In the following examples, we will discuss configurations that can be used to achieve the conceptually simple capability to maintain a safe distance from other aircraft in the airspace. In order for this capability to exist, there either needs to be one centralized location for general path planning for all vehicles in the airspace (e.g. air traffic control or NASA’s developing UTM technology) or the vehicles in the airspace must be able to communicate with each other via a peer-to-peer network.
 * 
 * \subsection centralized Centralized configuration
 * In the centralized case, each vehicle must be able to communicate with the centralized “server” their location and intended trajectory. Then, as vehicles update their positions and/or planned paths, the centralized “server” must re-plan paths and assign new targets for each vehicle, and then disseminate those new plans to each vehicle. In the decentralized case, it becomes much harder as individual vehicles may not be able to communicate with each vehicle in the airspace. Each vehicle must essentially keep a local copy of their knowledge of the airspace, and as new information about a vehicle is learned, update that copy. After updating their knowledge, they must update their plans accordingly to remain a safe distance from any vehicle they have knowledge of. 
 * 
 * MACE can help establish both of these cases while remaining agnostic to the specific vehicle architectures. First, let us start with the centralized example. Below is a high-level architecture diagram of how this may look in practice. 
 * 
 * \htmlonly <style>div.image img[src="centralized.png"]{width:700px;}</style> \endhtmlonly
 * \image html centralized.png width=2cm
 * \image latex centralized.eps "Centralized" width=10cm 
 * 
 * As the diagram outlines, each vehicle can have their own specific architecture and capabilities, but as long as they implement a MACE module, they can participate in a MACE network. In this case, each vehicle has their own way of determining their position and reporting it, whether it is a UTM specific piece of hardware or simply their own autopilot with an attached GPS. MACE does not care where the information comes from or how it is determined. What ultimately matters is that the MACE module for each vehicle is receiving position updates and intended trajectory updates from the vehicle. With that data, each vehicle’s MACE instance can then disseminate that to the centralized MACE instance for re-planning, if necessary. On the backend of this process, each vehicle then receives its new targets and the process iterates until mission completion or vehicles leave the airspace. 
 * 
 * \subsection decentralized Decentralized configuration
 * In the decentralized case, things become more complicated and more functionality is required from individual vehicles. However, the inter-vehicle communications is still handled by MACE. Instead of pushing all data through one centralized asset to conduct global planning, each vehicle is responsible for its own planning in its immediate airspace based on the knowledge it has available to it to keep a safe distance from other vehicles. This requires each vehicle have their own path planning and obstacle avoidance capabilities. The decentralized case is detailed in the figure below.
 * 
 * \htmlonly <style>div.image img[src="decentralized.png"]{width:700px;}</style> \endhtmlonly
 * \image html decentralized.png width=2cm
 * \image latex decentralized.eps "Centralized" width=10cm 
 * 
 * The core concept of the centralized case remains intact—i.e. each vehicle’s MACE module needs to receive position and intended trajectory updates from the attached vehicle and those need to be disseminated between each vehicle in the MACE network. However, it is not necessarily the case that each vehicle has knowledge of the complete MACE network and/or operating space. To get a complete picture, as MACE instances communicate with each other the local copy of the environment that each vehicle must be updated from new data coming from vehicles that may not be in direct communication. In addition, you cannot assume that when a vehicle plans a path to steer clear of vehicles it knows about that a vehicle it is not aware of will not interfere with that trajectory. That is where a vehicle’s local obstacle avoidance comes into play—it must take over the immediate planning horizon to avoid unknown obstacles that any intermediate or global knowledge of the world hasn’t accounted for yet. 
 * 
 *  */

#endif // DOXY_MAINPAGE_H
