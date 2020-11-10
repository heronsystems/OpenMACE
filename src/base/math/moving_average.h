#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#define _USE_MATH_DEFINES


#include <iostream>
#include <stddef.h>
#include <assert.h>
 
using std::cout;
using std::endl;
 
class MovingAverage {
public:
	MovingAverage(unsigned int period = 5) :
		period(period), window(new double[period]), head(NULL), tail(NULL),
				total(0) {
		assert(period >= 1);
	}
	~MovingAverage() {
		delete[] window;
	}
 
	// Adds a value to the average, pushing one out if nescessary
	void add(double val) {
		// Special case: Initialization
		if (head == NULL) {
			head = window;
			*head = val;
			tail = head;
			inc(tail);
			total = val;
			return;
		}
 
		if (head == tail) {
			// Fix total-cache
			total -= *head;
			// Make room
			inc(head);
		}
 
		*tail = val;
		inc(tail);
 
		// Update our total-cache
		total += val;
	}
 
	// Returns the average of the last P elements added.
	// If no elements have been added yet, returns 0.0
	double avg() const {
		ptrdiff_t size = this->size();
		if (size == 0) {
			return 0; // No entries => 0 average
		}
		return total / (double) size; // Cast to double for floating point arithmetic
	}
 
private:
	unsigned int period;
	double* window; // Holds the values to calculate the average of.
 
	// Logically, head is before tail
	double* head; // Points at the oldest element we've stored.
	double* tail; // Points at the newest element we've stored.
 
	double total; // Cache the total so we don't sum everything each time.
 
	// Bumps the given pointer up by one.
	// Wraps to the start of the array if needed.
	void inc(double * & p) {
		if (++p >= window + period) {
			p = window;
		}
	}
 
	// Returns how many numbers we have stored.
	ptrdiff_t size() const {
		if (head == NULL)
			return 0;
		if (head == tail)
			return period;
		return (period + tail - head) % period;
	}
};
#endif //MOVING_AVERAGE_H

