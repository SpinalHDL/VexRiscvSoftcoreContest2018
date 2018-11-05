#include <verilated_vcd_c.h>
#include <vector>


class Agent {
public:
    virtual void preCycle(uint64_t time) {}
    virtual void postCycle(uint64_t time) {}
};

template<class MODULE> class TESTBENCH {
public:
	// Need to add a new class variable
	VerilatedVcdC	*m_trace;
	uint64_t	time, tickCount;
	uint64_t	clkPeriod;
	MODULE	*dut;
	std::vector <Agent*> agents;

	TESTBENCH(uint64_t clkPeriod) : clkPeriod(clkPeriod) {
		Verilated::traceEverOn(true);
		dut = new MODULE();
		time = 0l;
        tickCount = 0l;
		#ifdef TRACE
		opentrace("wave.vcd");
		#endif
	}

	virtual ~TESTBENCH(void) {
		delete dut;
		dut = NULL;
	}

	void add(Agent *agent){
	    agents.push_back(agent);
	}

	// Open/create a trace file
    #ifdef TRACE
	virtual	void	opentrace(const char *vcdname) {
		if (!m_trace) {
			m_trace = new VerilatedVcdC;
			dut->trace(m_trace, 99);
			m_trace->open(vcdname);
		}
	}
    #endif

	// Close a trace file
    #ifdef TRACE
	virtual void	close(void) {
		if (m_trace) {
			m_trace->close();
			m_trace = NULL;
		}
	}
	#endif

	virtual void	reset(void) {
		dut->io_reset = 1;
		// Make sure any inheritance gets applied
		this->tick();
		dut->io_reset = 0;
	}

	virtual void	tick(void) {
		// Make sure the tickcount is greater than zero before
		// we do this

		// Allow any combinatorial logic to settle before we tick
		// the clock.  This becomes necessary in the case where
		// we may have modified or adjusted the inputs prior to
		// coming into here, since we need all combinatorial logic
		// to be settled before we call for a clock tick.
		//
		dut->io_clk = 0;
		dut->eval();

		// Repeat for the positive edge of the clock
		dut->io_clk = 1;
		for(auto agent : agents) agent->preCycle(time);
		dut->eval();
        for(auto agent : agents) agent->postCycle(time);

        #ifdef TRACE
		if(m_trace) m_trace->dump(time);
		#endif

		// Now the negative edge
		time += clkPeriod/2;
		dut->io_clk = 0;
		dut->eval();

        #ifdef TRACE
		if (m_trace) {
			// This portion, though, is a touch different.
			// After dumping our values as they exist on the
			// negative clock edge ...
			m_trace->dump(time);
			//
			// We'll also need to make sure we flush any I/O to
			// the trace file, so that we can use the assert()
			// function between now and the next tick if we want to.
			m_trace->flush();
		}
		#endif
		time += clkPeriod/2;
        tickCount++;
	}

	virtual bool	done(void) { return (Verilated::gotFinish()); }
};


