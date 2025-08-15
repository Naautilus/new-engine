#pragma once
#include "../vector/vector_spaces.cpp"
#include "../constants/constants.cpp"
#include "../physics_object/object.cpp"
#include "../physics_object/blueprints.cpp"
#include "collider.cpp"
#include "../math/random.cpp"
#include "collision_data.cpp"

namespace collision {

	void create_debris_for_objects(physics_object::object& a, physics_object::object& b, double desired_debris_mass, vector::worldspace& collision_point) {
		if (desired_debris_mass < 10) return;
	
		const double DIRECTION_RANDOMIZATION = 0.03;
		const double MAGNITUDE_MEAN = 0.9; // values >1 will be recalculated
		const double MAGNITUDE_STDEV = 0.05;
		const double FRAGMENT_SPREAD_DIRECTIONALITY = 0.4; // 0 is spread evenly between the two colliding objects, 1 is heavily biased towards the heavier object
		const double FRAGMENT_SPREAD_DIRECTION_1_CHANCE = 0.7;
		const double FRAGMENT_PARTICULATE_WIDTH_EXPONENT = -0.8; // around -1.0 to -0.8 in real data, but that's for space and not a burning wreck
		std::normal_distribution<double> random_velocity_offset(0, DIRECTION_RANDOMIZATION);
		std::normal_distribution<double> random_velocity_magnitude(MAGNITUDE_MEAN, MAGNITUDE_STDEV);
		std::normal_distribution<double> random_interpolation(0.5, 0.15);
	
		std::vector<double> debris_masses;
		double debris_count = desired_debris_mass/8;//desired_debris_mass/15;
		//double debris_count = desired_debris_mass*10.0/8;//desired_debris_mass/15;
		for (int i = 0; i < debris_count; i++) {
			double debris_rarity = 1 + (i / debris_count) * 1e2;
			double debris_mass = pow(debris_rarity, FRAGMENT_PARTICULATE_WIDTH_EXPONENT);
			debris_masses.push_back(debris_mass);
		}
		double total_mass_original = 0;
		for (double mass : debris_masses) total_mass_original += mass;
		double mass_factor = desired_debris_mass / total_mass_original;
		for (double& mass : debris_masses) mass *= mass_factor;
	
		for (double mass : debris_masses) {
		
			double object_weight_ratio = a.physics_state.mass / b.physics_state.mass;
			if (object_weight_ratio < 1) object_weight_ratio = 1 / object_weight_ratio;
			double interp = std::clamp(random_interpolation(globals::rng), 0.0, 1.0);
			if (random(0.0, 1.0) > FRAGMENT_SPREAD_DIRECTION_1_CHANCE) {
			  	interp = pow(interp, pow(object_weight_ratio, FRAGMENT_SPREAD_DIRECTIONALITY)); // direction 1 - the fragments directed towards the bigger vehicle; should be more likely
			} else {
			  	interp = pow(interp, pow(object_weight_ratio, -FRAGMENT_SPREAD_DIRECTIONALITY)); // direction 2 - the fragments directed towards the bigger vehicle
			}
		
			//vector::worldspace position = (1-interp) * a.physics_state.position + interp * b.physics_state.position;
            vector::worldspace position = collision_point;
			vector::worldspace velocity = (1-interp) * a.physics_state.velocity + interp * b.physics_state.velocity;
			//std::cout << mass << std::endl;
            auto d = std::make_shared<physics_object::object>(physics_object::blueprints::debris(mass));
			//std::cout << "position: " << position.transpose() << std::endl;
			//std::cout << "velocity: " << velocity.transpose() << std::endl;
			d->physics_state.position = position;
			d->physics_state.velocity = velocity;
		
			vector::worldspace velocity_offset;
			do {velocity_offset = vector::worldspace(random(-1,1), random(-1,1), random(-1,1));} while (velocity_offset.squaredNorm() > 1);
			velocity_offset /= velocity_offset.norm();
			velocity_offset *= random_velocity_offset(globals::rng);
			velocity_offset *= velocity.norm();
			d->physics_state.velocity += velocity_offset;
			double velocity_multiplier = random_velocity_magnitude(globals::rng);
			d->physics_state.velocity *= velocity_multiplier;
		
            globals::physics_objects_mutex.lock();
			globals::physics_objects.push_back(d);
            globals::physics_objects_mutex.unlock();
		
		}
	}
	
	void process_colliding_physics_objects(collision::collider& a_collider, collision::collider& b_collider, physics_object::object& a, physics_object::object& b) {
		//std::cout << "process_colliding_physics_objects called\n";
        std::optional<collision_data> collision_data_optional = a_collider.get_collision_data(b_collider);
        if (!collision_data_optional) {
            //std::cout << "process_colliding_physics_objects(...): no collision point found\n";
            return;
        }
        vector::worldspace collision_point = collision_data_optional.value().position;
        if (std::isnan(collision_point.squaredNorm())) return;
        //std::cout << "point of collision: " << collision_point.transpose() << "\n";

        /*
        globals::physics_objects_mutex.lock();
        globals::physics_objects.push_back(physics_object::blueprints::axes(collision_point));		
        globals::physics_objects_mutex.unlock();
        */

        Eigen::Matrix3d impulse_response_per_axis;
        vector::worldspace axes[3] = {
            vector::worldspace(1, 0, 0),
            vector::worldspace(0, 1, 0),
            vector::worldspace(0, 0, 1)
        };
        
        physics_state original_physics_state_a = a.physics_state;
        physics_state original_physics_state_b = b.physics_state;

        vector::worldspace delta_velocity = b.physics_state.velocity_at_point(collision_point) - a.physics_state.velocity_at_point(collision_point);
        
        for (int i = 0; i < 3; i++) {
            a.apply_impulse(collision_point,  axes[i]);
            b.apply_impulse(collision_point, -axes[i]);
            impulse_response_per_axis.row(i) = (b.physics_state.velocity_at_point(collision_point) - a.physics_state.velocity_at_point(collision_point)) - delta_velocity;
            a.physics_state = original_physics_state_a;
            b.physics_state = original_physics_state_b;
        }
        
        vector::worldspace result = impulse_response_per_axis.colPivHouseholderQr().solve(-1 * delta_velocity);
        
        /*
        std::cout << "solving for Ax = b. A:\n";
        std::cout << impulse_response_per_axis << "\n";
        std::cout << "b:\n";
        std::cout << -delta_velocity << "\n";
        std::cout << "x:\n";
        std::cout << result << "\n";
        std::cout << "\n";
        
        std::cout << "impulse response x: " << impulse_response_per_axis.row(0) << "\n";
        std::cout << "impulse response y: " << impulse_response_per_axis.row(1) << "\n";
        std::cout << "impulse response z: " << impulse_response_per_axis.row(2) << "\n";
        std::cout << "original delta_velocity: " << delta_velocity.transpose() << "\n";
        std::cout << "resulting impulse: " << result.transpose() << "\n";
        */

        a.apply_impulse(collision_point,  result);
        b.apply_impulse(collision_point, -result);

        //std::cout << "new delta_velocity: " << (b.physics_state.velocity_at_point(collision_point) - a.physics_state.velocity_at_point(collision_point)).transpose() << "\n";

        // if colliding:
        double minimum_mass = fmin(a.physics_state.mass, b.physics_state.mass);
        double damage = constants::DAMAGE_MULTIPLIER * minimum_mass * 
                        fmax(delta_velocity.squaredNorm() - constants::SAFE_COLLISION_SPEED * constants::SAFE_COLLISION_SPEED, 0);
        double maximum_health = fmax(a.physics_state.health, b.physics_state.health);
        damage = fmin(damage, maximum_health);
        
        a.physics_state.health -= damage;
        b.physics_state.health -= damage;

        std::cout << "damage: " << damage << "\n";

        double total_mass = a.physics_state.mass + b.physics_state.mass;

        /*
        if (a.physics_state.health > 0 && b.physics_state.health > 0) {
            while (a_collider.check_collision(b_collider)) {
                std::cout << "push iteration\n";
                a.physics_state.position += 0.1 * delta_velocity * constants::DELTA_T * (b.physics_state.mass / total_mass);
                b.physics_state.position -= 0.1 * delta_velocity * constants::DELTA_T * (a.physics_state.mass / total_mass);
                a_collider.update(a.physics_state.position, a.physics_state.velocity, a.physics_state.rotation);
                b_collider.update(b.physics_state.position, b.physics_state.velocity, b.physics_state.rotation);
            }
        }
        */

        create_debris_for_objects(a, b, damage, collision_point);
    }
        
	double impact_velocity(vector::worldspace& velocity, vector::worldspace& ground_normal) {
        return fabs(ground_normal.dot(velocity));
	}
	
	double angle_of_incidence(vector::worldspace& velocity, vector::worldspace& ground_normal) {
        double dot = ground_normal.dot(velocity) / velocity.norm();
		double angle = acos(-dot);
		angle *= 180.0 / std::numbers::pi;
		return angle;
	}
	
    collider collider_representing_ground(collider& c) {
        vector::worldspace position = c.position;
        double width = 2 * sqrt(c.bounding_box_width_squared);
        //std::cout << "width: " << width << "\n";
        position.z() = ground::get_ground_altitude(position.x(), position.y());
        vector::worldspace normal = ground::get_surface_normal(position.x(), position.y());
        vector::worldspace x = normal.cross(vector::worldspace(0, 1, 0));
        vector::worldspace y = normal.cross(x);
        x *= width;
        y *= width;
        vector::worldspace p0 = position + x;
        vector::worldspace p1 = position + -0.5*x + 0.866*y;
        vector::worldspace p2 = position + -0.5*x - 0.866*y;
        collider output = collider({triangle(p0, p1, p2)});
        output.update(
            vector::worldspace(0, 0, 0),
            vector::worldspace(0, 0, 0),
            Eigen::Quaterniond::Identity()
        );

        /*
        std::cout << "collider representing ground created\n";
        std::cout << "position:" << position.transpose() << "\n";
        std::cout << "p0:" << p0.transpose() << "\n";
        std::cout << "p1:" << p1.transpose() << "\n";
        std::cout << "p2:" << p2.transpose() << "\n";
        */

        return output;
    }
    
	void process_ground_collision(physics_object::object& o) {
	
        physics_object::object ground_object = physics_object::object();
        ground_object.physics_state.mass = 6e24;
        ground_object.physics_state.health = 6e24;
        
        if (!o.properties.functional) {
            double altitude = o.physics_state.position.z() - ground::get_ground_altitude(o.physics_state.position.x(), o.physics_state.position.y());
            if (altitude < 0) o.physics_state.health = 0;
            return;
        }
        for (std::shared_ptr<module::module>& m : o.properties.modules) {
            collider ground_collider = collider_representing_ground(m->collider);
            if (!m->collider.check_collision(ground_collider)) continue;
            process_colliding_physics_objects(m->collider, ground_collider, o, ground_object);
            //o.physics_state.position += vector::worldspace(0, 0, 0.01);
        }
        
		
		/*
        globals::physics_objects_mutex.lock();
        globals::physics_objects.push_back(physics_object::blueprints::collider_visual(vector::worldspace(0, 0, 0), ground_collider));
        globals::physics_objects_mutex.unlock();
		*/


        /*
		double altitude = o.physics_state.position.z() - ground::get_ground_altitude(o.physics_state.position.x(), o.physics_state.position.y());
		if (altitude > 5) return;
		o.physics_state.velocity *= pow(0.5, constants::DELTA_T);
		if (altitude > 0) return;
		
		vector::worldspace ground_normal = ground::get_surface_normal(o.physics_state.position.x(), o.physics_state.position.y());
		vector::worldspace velocity_planar = o.physics_state.velocity - ground_normal*ground_normal.dot(o.physics_state.velocity);
		vector::worldspace velocity_normal = -ground_normal*ground_normal.dot(o.physics_state.velocity);
		
		if (!o.properties.functional && o.physics_state.velocity.squaredNorm() < 3*3) {
		  	o.physics_state.velocity *= 0.1;
		  	o.physics_state.angular_velocity *= 0.1;
		  	o.physics_state.position.z() += 0.01 - altitude;
		  	return;
		}
	
		if (angle_of_incidence(o.physics_state.velocity, ground_normal) > 80.0 && o.physics_state.mass <= 50.0) { // little objects <50kg can ricochet
		  	o.physics_state.velocity = 0.4*velocity_planar + 0.8*velocity_normal;
		  	o.physics_state.position.z() += 0.01 - altitude;
		  	return;
		}
		
		if (impact_velocity(o.physics_state.velocity, ground_normal) > 20.0) {
		  	physics_object::object b;
		  	b.physics_state.mass = o.physics_state.mass * 1;
		  	b.physics_state.position = o.physics_state.position;
		  	o.physics_state.velocity = velocity_planar + 2 * velocity_normal;
		  	//b.physics_state.velocity = o.physics_state.velocity;
		  	create_debris_for_objects(b, o, o.physics_state.health);
		  	o.physics_state.health = 0;
		  	return;
		}
		
		o.physics_state.velocity = velocity_planar;
		o.physics_state.position.z() += 0.01 - altitude;
        */
		
	}

}