/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2017 Intel Corporation. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */
/*! @file
 *  This file contains an ISA-portable cache simulator
 *  instruction cache hierarchies
 */


#include "pin.H"

#include <iostream>
#include <fstream>
#include <cassert>

#include <stack>

#include "cache.H"
#include "pin_profile.H"


/* ===================================================================== */
/* Commandline Switches */
/* ===================================================================== */

using std::cerr;
using std::string;
using std::endl;
using namespace std;
#define _THREADID 15
#define DEGREE_OF_USE 1.5
#define MEDIUM_DEGREE_OF_USE 1.0
#define INSTRUCTION_THRESHOLD 500000000
#define MISS_PER_FUNCTION_THRESHOLD 15.0
//#define ACTIVE_LOW_FUNCTION_LOGGING 0
//#define PERLBENCH_DEBUG 0

/* ===================================================================== */

uint64_t total_misses = 0;
uint64_t count_misses_from_low_degree_functions = 0;
uint64_t count_misses_from_high_degree_functions = 0;

uint64_t count_missses_from_low_degree_functions_after_call = 0;

uint64_t count_misses_from_medium_degree_functions = 0;

uint64_t count_misses_from_low_degree_functions_normal_cache = 0;

uint64_t count_missses_from_low_degree_functions_normal_cache_after_call = 0;

uint64_t count_misses_from_high_degree_functions_normal_cache = 0;
uint64_t count_misses_from_medium_degree_functions_normal_cache = 0;

uint64_t count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions = 0;
uint64_t count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade = 0;
uint64_t count_of_blocks_displaced_from_high_use_functions_by_low_use_two_functions = 0;
uint64_t count_of_blocks_displaced_from_high_use_functions_by_high_use_functions = 0;
uint64_t count_of_low_use_displacing_low_use_functions = 0;
uint64_t count_of_low_use_allocated_way0 = 0;
uint64_t total_misses_on_low_use_functions = 0;
set<uint64_t> functions_with_low_use;
set<uint64_t> total_functions;

bool done = false;
bool enable_instrumentation = true;

KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE,    "pintool",
    "o", "icache_32k_fg_http_deg2_lru_position_victim8entry.out", "specify icache file name");
KNOB<BOOL>   KnobTrackInsts(KNOB_MODE_WRITEONCE,    "pintool",
    "ti", "0", "track individual instructions -- increases profiling time");
KNOB<UINT32> KnobThresholdHit(KNOB_MODE_WRITEONCE , "pintool",
    "rh", "100", "only report ops with hit count above threshold");
KNOB<UINT32> KnobThresholdMiss(KNOB_MODE_WRITEONCE, "pintool",
    "rm","100", "only report ops with miss count above threshold");
KNOB<UINT32> KnobCacheSize(KNOB_MODE_WRITEONCE, "pintool",
    "c","32", "cache size in kilobytes");
KNOB<UINT32> KnobLineSize(KNOB_MODE_WRITEONCE, "pintool",
    "b","64", "cache block size in bytes");
KNOB<UINT32> KnobAssociativity(KNOB_MODE_WRITEONCE, "pintool",
    "a","8", "cache associativity (1 for direct mapped)");

KNOB<UINT32> KnobITLBSize(KNOB_MODE_WRITEONCE, "pintool",
    "ci","32", "cache size in kilobytes");
KNOB<UINT32> KnobITLBLineSize(KNOB_MODE_WRITEONCE, "pintool",
    "bi","64", "cache block size in bytes");
KNOB<UINT32> KnobITLBAssociativity(KNOB_MODE_WRITEONCE, "pintool",
                "ai","8", "cache associativity (1 for direct mapped)");



#define MISS_THRESHOLD 50
#define INVOCATION_THRESHOLD 50
INT32 Usage()
{
    cerr <<
        "This tool represents a cache simulator.\n"
        "\n";

    cerr << KNOB_BASE::StringKnobSummary();

    cerr << endl;

    return -1;
}

bool call_instr_seen = false;
bool dir_jump_instr_seen = false;
int64_t prev_dir_jump_page;
bool syscall_seen = false;
bool ind_call_instr_seen = false;
bool return_instr_seen = false;
bool ind_jump_seen = false;
int64_t prev_ind_jump_page;
uint64_t prev_ind_jump_iaddr;
/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */

// wrap configuation constants into their own name space to avoid name clashes
namespace IL1
{
    const UINT32 max_sets = KILO; // cacheSize / (lineSize * associativity);
    const UINT32 max_associativity = 256; // associativity;
    const CACHE_ALLOC::STORE_ALLOCATION allocation = CACHE_ALLOC::STORE_ALLOCATE;
    
    typedef CACHE_ROUND_ROBIN(max_sets, max_associativity, allocation) CACHE;
}


// wrap configuation constants into their own name space to avoid name clashes
namespace ITLB
{
    const UINT32 max_sets = KILO*8; // cacheSize / (lineSize * associativity);
    const UINT32 max_associativity = 256; // associativity;
    const CACHE_ALLOC::STORE_ALLOCATION allocation = CACHE_ALLOC::STORE_ALLOCATE;

    typedef CACHE_MODIFIED_CACHE(max_sets, max_associativity, allocation) CACHE;
}

struct page_and_cache_block {
    uint64_t x, y;
    page_and_cache_block() {}
    page_and_cache_block (int _x, int _y) {
        x = _x;
        y = _y;
    }
    bool operator<(const page_and_cache_block &rhs) const{
        return make_pair(y,x) < make_pair(rhs.y, rhs.x);
    }
    bool operator==(const page_and_cache_block &rhs) const{
        return make_pair(y,x) == make_pair(rhs.y, rhs.x);
    }
};

struct function_stats{
	set<uint64_t> unique_cache_blocks_touched_by_function;
	uint64_t func_miss_count;
	uint64_t func_total_itlb_miss_count;
	uint64_t func_total_miss_count;
	uint64_t func_invocation_count;
	//function classified as low use function. 
	bool low_degree_function;
	bool medium_degree_function;
	bool initialized;
};

//maintain this per callee address or per cache block. 
map<uint64_t, function_stats> function_invocation_count;


ITLB::CACHE* itlb = NULL;


IL1::CACHE* il1 = NULL;

//datastructures used to note the number of cache blocks
//that are constitute a function. 
stack <uint64_t> call_stack;
map<page_and_cache_block, set<uint64_t>> mapping_from_function_to_number_of_cache_blocks;
//current function identified by the cache block
//that the callee address is a part of
page_and_cache_block current_function(0,0);
uint64_t current_function_callee_address = 1;
uint64_t current_cache_block;
set<uint64_t> number_of_active_low_use_functions;
vector<uint64_t> list_of_active_low_use_function_counts;

//number of cache blocks part of a function in perlbench. 
set<uint64_t> number_of_cache_blocks_part_of_function_of_interest_perlbench;
uint64_t instructions_spent_in_function_of_interest = 0;
bool recorded = false;


typedef enum
{
    COUNTER_MISS = 0,
    COUNTER_HIT = 1,
    COUNTER_NUM
} COUNTER;


//counters to count the number of itlb misses happening after different kinds of 
//global control transfer instructions.

uint64_t itlb_misses_after_call = 0;
uint64_t icache_misses_after_ind_jump = 0;
uint64_t itlb_misses_after_return = 0;
uint64_t itlb_misses_after_syscall = 0;
uint64_t itlb_misses_after_none_of_above = 0;

uint64_t icache_misses_after_long_jump = 0;

//cache misses from shared library
uint64_t icache_misses_from_shared_library = 0;

typedef  COUNTER_ARRAY<UINT64, COUNTER_NUM> COUNTER_HIT_MISS;

set<uint64_t> list_of_high_use_blocks_replaced;



// holds the counters with misses and hits
// conceptually this is an array indexed by instruction address
COMPRESSOR_COUNTER<ADDRINT, UINT32, COUNTER_HIT_MISS> profile;

/* ===================================================================== */

VOID LoadMulti(ADDRINT addr, UINT32 size, UINT32 instId)
{
    // first level I-cache
    const BOOL il1Hit = il1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);

    const COUNTER counter = il1Hit ? COUNTER_HIT : COUNTER_MISS;
    profile[instId][counter]++;
}

/* ===================================================================== */

VOID LoadSingle(ADDRINT addr, UINT32 instId)
{
    // @todo we may access several cache lines for 
    // first level I-cache
    const BOOL il1Hit = il1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);

    const COUNTER counter = il1Hit ? COUNTER_HIT : COUNTER_MISS;
    profile[instId][counter]++;
}

/* ===================================================================== */

VOID LoadMultiFast(ADDRINT addr, UINT32 size, THREADID tid)
{

   if (tid == _THREADID){ 
       //first step is to identify the function we are executing, sometimes we might jump out to function 
       //to run another function and then get back to executing a function. This necessitates the use of call stack
       //to identify the function we are executing.  
       //uint64_t cache_block_addr;
       //cache_block_addr = addr/64;
       //if we access a new cache block, then we record this cache block as part of the current function. 
       //work with the assumption a function call involves access of a new cache block.
       //if (cache_block_addr != current_cache_block){
       	  if (call_instr_seen){
	    call_stack.push(current_function_callee_address);
	    current_function_callee_address = addr;
#ifdef ACTIVE_LOW_FUNCTION_LOGGING 
	    //whenever a low use function becomes active, make a note
	    if (function_invocation_count[current_function_callee_address].low_degree_function)
		number_of_active_low_use_functions.insert(current_function_callee_address);
#endif 
	  }
	  else if(return_instr_seen){
	    if (call_stack.size()!=0){
		current_function_callee_address = call_stack.top();
		call_stack.pop();
	    }
	  }
	 function_invocation_count[current_function_callee_address].unique_cache_blocks_touched_by_function.insert(addr/64); 
	 uint64_t number_of_function_misses = 0;
	 uint64_t number_of_function_invocations = 0; 
         if (function_invocation_count.find(current_function_callee_address) == 
          		function_invocation_count.end()){
         	number_of_function_misses = 0;
        	function_invocation_count[current_function_callee_address].func_miss_count = 0;
                function_invocation_count[current_function_callee_address].func_total_miss_count = 0;
		function_invocation_count[current_function_callee_address].func_invocation_count = 0;
		function_invocation_count[current_function_callee_address].low_degree_function = false;
		function_invocation_count[current_function_callee_address].medium_degree_function = false;
		function_invocation_count[current_function_callee_address].initialized = true;
	 }
	 else{
	 	number_of_function_misses = function_invocation_count[current_function_callee_address].func_miss_count;
	        number_of_function_invocations = function_invocation_count[current_function_callee_address].func_invocation_count;	
	 }
	 float degree_of_use;
	 if (number_of_function_misses == 0)
		 number_of_function_misses = 1;
	 degree_of_use = (float)number_of_function_invocations/number_of_function_misses;
	 hit_and_use_information temp,temp1;
       
     	 //set the degree of use flag to true for code
       //from functions with a high degree of use. 
       //because degree of use affects placement in the cache, allow for a few misses before we start to place functions
       //assuming they are a low use function.  
       
	 bool degree_of_use_bool;
	 if (degree_of_use<= DEGREE_OF_USE){
		degree_of_use_bool = false;
		//classify the function once and for all as low use, because otherwise
		//function's class might change to high use and again start to interfere 
		//with high use functions, which we want to avoid. 
	// 	float misses_per_function = ((float)function_invocation_count[current_function_callee_address].func_total_miss_count/
	//						function_invocation_count[current_function_callee_address].func_miss_count);
		if ((number_of_function_misses>= MISS_THRESHOLD) && 
			       // (misses_per_function<= MISS_PER_FUNCTION_THRESHOLD) &&	
				(!function_invocation_count[current_function_callee_address].low_degree_function)){
		   //check if the function has the medim degree of use, and if yes, set the additional medium degree of use
		   //flag.
		   if (degree_of_use > MEDIUM_DEGREE_OF_USE)
		       function_invocation_count[current_function_callee_address].medium_degree_function = true;	   
		   function_invocation_count[current_function_callee_address].low_degree_function = true;
		}	       
	 }
	 //if a function goes from being a low use function to 
	 //seeing more use, then check and revert the low degree function flag.
	 else{
		 degree_of_use_bool = true;
	// 	if (function_invocation_count[current_function_callee_address].low_degree_function)
	//		function_invocation_count[current_function_callee_address].low_degree_function = false;
	 }
	 if ((degree_of_use_bool)||(number_of_function_misses<= MISS_THRESHOLD))
	 	temp = il1->Access_selective_allocate(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD, true, true, false, false);
	 else
       		temp = il1->Access_selective_allocate(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD, true, false, false, false);
	 if (function_invocation_count[current_function_callee_address].low_degree_function){
		 bool medium_degree_of_use = false;
		 //if (function_invocation_count[current_function_callee_address].medium_degree_function)
			 medium_degree_of_use = true;
		 temp1 = itlb->Access_selective_allocate(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD, true, false, medium_degree_of_use, false);
	 }
	 else
       		temp1 = itlb->Access_selective_allocate(addr, size,  CACHE_BASE::ACCESS_TYPE_LOAD, true, true, false, false);
       
	 if (!temp.icache_hit)
		total_misses++;
	 if (!temp1.icache_hit){
	 total_misses_on_low_use_functions = temp1.total_low_use_misses;
	 }
	 //count number of misses coming from function invoked a lot and which are not low use. 
	 if (((!function_invocation_count[current_function_callee_address].low_degree_function)&&
				 (function_invocation_count[current_function_callee_address].func_invocation_count>=INVOCATION_THRESHOLD)
				 &&(!temp1.icache_hit))){
	    count_misses_from_high_degree_functions++; 
	    //if replaced block was from a high use function, then this flag would be set.
            
    	    const ADDRINT notLineMask = ~(64 - 1);
	    uint64_t current_cache_block_address = addr&notLineMask;
	    if (temp1.function_use_information){
	      count_of_blocks_displaced_from_high_use_functions_by_high_use_functions++;	 
	      //if current block was replaced by a low use function or in a cascade of misses
	      //following the miss from a low use function. 
	      if (list_of_high_use_blocks_replaced.find(current_cache_block_address) !=
			      list_of_high_use_blocks_replaced.end()){
		  //add all the high use code we replace to the list of blocks replaced part of 
		  //the cascade. 
	      	  count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade += 
		     temp1.blk_addresses.size(); 
		  for (uint64_t i = 0;i<temp1.blk_addresses.size();i++) 
			 list_of_high_use_blocks_replaced.insert(temp1.blk_addresses.at(i));
	      
	      }
	    }
	 }
	 else if (((function_invocation_count[current_function_callee_address].low_degree_function)&&
				 (!temp1.icache_hit))){
	    count_misses_from_low_degree_functions++; 
	    if (call_instr_seen)
		count_missses_from_low_degree_functions_after_call++;
	    //if replaced block was from a high use function, then this flag would be set.
	    if (temp1.function_use_information){
		count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions++;		
	        count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade += 
			temp1.blk_addresses.size();
		for (uint64_t i = 0;i<temp1.blk_addresses.size();i++) 
			list_of_high_use_blocks_replaced.insert(temp1.blk_addresses.at(i));
	    }
	    else
		count_of_low_use_displacing_low_use_functions++;
	    if (temp1.allocated_way == 0)
		count_of_low_use_allocated_way0+= 1;
	    functions_with_low_use.insert(current_function_callee_address); 
	 }
	 if (((!function_invocation_count[current_function_callee_address].low_degree_function) &&(function_invocation_count[current_function_callee_address].func_invocation_count>=INVOCATION_THRESHOLD)
		&&(!temp.icache_hit)))
	     count_misses_from_high_degree_functions_normal_cache++;
	 else if (((function_invocation_count[current_function_callee_address].low_degree_function)&&(!temp.icache_hit))){
	     count_misses_from_low_degree_functions_normal_cache++;
	     if (call_instr_seen)
		     count_missses_from_low_degree_functions_normal_cache_after_call++;
	 }
	 if (!temp1.icache_hit){
    	     const ADDRINT notLineMask = ~(64 - 1);
	     uint64_t current_cache_block_address = addr&notLineMask;
	     //remove current block from the list of high use blocks, because we have already counted
	     //its removal once and have also accounted for any cascade if any above. 
	     list_of_high_use_blocks_replaced.erase(current_cache_block_address);	
	 }
////	  
//////       }	
        if (!temp1.icache_hit){
           if (call_instr_seen){
        		if ((function_invocation_count.find(current_function_callee_address) != function_invocation_count.end())){
		   		function_invocation_count[current_function_callee_address].func_miss_count++;
        			function_invocation_count[current_function_callee_address].func_total_miss_count++;
				function_invocation_count[current_function_callee_address].func_invocation_count++;
			}
           }
	   else
	    function_invocation_count[current_function_callee_address].func_total_miss_count++;
	}
        else{
	  if (call_instr_seen)     
           function_invocation_count[current_function_callee_address].func_invocation_count++;
        }
//       if (!temp1.icache_hit){
//          if (call_instr_seen)     
//		function_invocation_count[current_function_callee_address].func_total_itlb_miss_count++;	
//       }
       call_instr_seen = false;
       ind_jump_seen = false;
       return_instr_seen = false;
       syscall_seen = false;
       dir_jump_instr_seen = false;
   }	
//	il1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);
}

/* ===================================================================== */

VOID LoadSingleFast(ADDRINT addr, THREADID tid)
{
   if (tid == _THREADID){ 
       //first step is to identify the function we are executing, sometimes we might jump out to function 
       //to run another function and then get back to executing a function. This necessitates the use of call stack
       //to identify the function we are executing.  
       //uint64_t cache_block_addr;
       //cache_block_addr = addr/64;
       //if we access a new cache block, then we record this cache block as part of the current function. 
       //work with the assumption a function call involves access of a new cache block.
       //if (cache_block_addr != current_cache_block){
       	  if (call_instr_seen){
            call_stack.push(current_function_callee_address);
            current_function_callee_address = addr;
          }
          else if(return_instr_seen){
            if (call_stack.size()!=0){
        	current_function_callee_address = call_stack.top();
        	call_stack.pop();
            }
          }
         
	 function_invocation_count[current_function_callee_address].unique_cache_blocks_touched_by_function.insert(addr/64); 
         uint64_t number_of_function_misses = 0;
         uint64_t number_of_function_invocations = 0; 
         if (function_invocation_count.find(current_function_callee_address) == 
          		function_invocation_count.end()){
         	number_of_function_misses = 0;
        	function_invocation_count[current_function_callee_address].func_miss_count = 0;
                function_invocation_count[current_function_callee_address].func_total_miss_count = 0;
		function_invocation_count[current_function_callee_address].func_invocation_count = 0;
		function_invocation_count[current_function_callee_address].low_degree_function = false;
		function_invocation_count[current_function_callee_address].medium_degree_function = false;
		function_invocation_count[current_function_callee_address].initialized = true;
         }
         else{
         	number_of_function_misses = function_invocation_count[current_function_callee_address].func_miss_count;
                number_of_function_invocations = function_invocation_count[current_function_callee_address].func_invocation_count;	
         }
         float degree_of_use;
         if (number_of_function_misses == 0)
        	 number_of_function_misses = 1;
         degree_of_use = (float)number_of_function_invocations/number_of_function_misses;
         hit_and_use_information temp,temp1;
         
       	 //set the degree of use flag to true for code
         //from functions with a high degree of use. 
         //because degree of use affects placement in the cache, allow for a few misses before we start to place functions
         //assuming they are a low use function.  
         
         bool degree_of_use_bool;
         if (degree_of_use<= DEGREE_OF_USE){
        	degree_of_use_bool = false;
        	//classify the function once and for all as low use, because otherwise
        	//function's class might change to high use and again start to interfere 
        	//with high use functions, which we want to avoid. 
        // 	float misses_per_function = ((float)function_invocation_count[current_function_callee_address].func_total_miss_count/
        //						function_invocation_count[current_function_callee_address].func_miss_count);
        	if ((number_of_function_misses>= MISS_THRESHOLD) && 
        		      //  (misses_per_function<= MISS_PER_FUNCTION_THRESHOLD) &&	
        			(!function_invocation_count[current_function_callee_address].low_degree_function)){
        	       
		   //check if the function has the medim degree of use, and if yes, set the additional medium degree of use
		   //flag.
		   	if (degree_of_use > MEDIUM_DEGREE_OF_USE)
		       		function_invocation_count[current_function_callee_address].medium_degree_function = true;	   
		   	function_invocation_count[current_function_callee_address].low_degree_function = true;	
		}
         }
	 //if a function goes from being a low use function to 
	 //seeing more use, then check and revert the low degree function flag.
	 else{
		degree_of_use_bool = true;
	// 	if (function_invocation_count[current_function_callee_address].low_degree_function)
	//		function_invocation_count[current_function_callee_address].low_degree_function = false;
	 }
         if ((degree_of_use_bool)||(number_of_function_misses<= MISS_THRESHOLD))
         	temp = il1->AccessSingleLine_selective_allocate(addr, CACHE_BASE::ACCESS_TYPE_LOAD, true, true, false, false);
         else
         	temp = il1->AccessSingleLine_selective_allocate(addr, CACHE_BASE::ACCESS_TYPE_LOAD, true, false, false, false);
         if (function_invocation_count[current_function_callee_address].low_degree_function){
		 bool medium_degree_of_use = false;
		 //if (function_invocation_count[current_function_callee_address].medium_degree_function)
			 medium_degree_of_use = true;
         	 temp1 = itlb->AccessSingleLine_selective_allocate(addr, CACHE_BASE::ACCESS_TYPE_LOAD, true, false, medium_degree_of_use, false);
	 }
         else
         	temp1 = itlb->AccessSingleLine_selective_allocate(addr,  CACHE_BASE::ACCESS_TYPE_LOAD, true, true, false, false);
         
         if (!temp.icache_hit)
        	total_misses++;
         if (!temp1.icache_hit){
         total_misses_on_low_use_functions = temp1.total_low_use_misses;
         }
         //count number of misses coming from function invoked a lot and which are not low use. 
         if (((!function_invocation_count[current_function_callee_address].low_degree_function)&&
        			 (function_invocation_count[current_function_callee_address].func_invocation_count>=INVOCATION_THRESHOLD)
        			 &&(!temp1.icache_hit))){
            count_misses_from_high_degree_functions++; 
            //if replaced block was from a high use function, then this flag would be set.
            
    	    const ADDRINT notLineMask = ~(64 - 1);
            uint64_t current_cache_block_address = addr&notLineMask;
            if (temp1.function_use_information){
              count_of_blocks_displaced_from_high_use_functions_by_high_use_functions++;	 
              //if current block was replaced by a low use function or in a cascade of misses
              //following the miss from a low use function. 
              if (list_of_high_use_blocks_replaced.find(current_cache_block_address) !=
        		      list_of_high_use_blocks_replaced.end()){
        	  //add all the high use code we replace to the list of blocks replaced part of 
        	  //the cascade. 
              	  count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade += 
        	     temp1.blk_addresses.size(); 
        	  for (uint64_t i = 0;i<temp1.blk_addresses.size();i++) 
        		 list_of_high_use_blocks_replaced.insert(temp1.blk_addresses.at(i));
              
              }
            }
         }
         else if (((function_invocation_count[current_function_callee_address].low_degree_function)&&
        			 (!temp1.icache_hit))){
            count_misses_from_low_degree_functions++; 
	    if (call_instr_seen)
		count_missses_from_low_degree_functions_after_call++;
	    //if replaced block was from a high use function, then this flag would be set.
            if (temp1.function_use_information){
        	count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions++;		
                count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade += 
        		temp1.blk_addresses.size();
        	for (uint64_t i = 0;i<temp1.blk_addresses.size();i++) 
        		list_of_high_use_blocks_replaced.insert(temp1.blk_addresses.at(i));
            }
            else
        	count_of_low_use_displacing_low_use_functions++;
            if (temp1.allocated_way == 0)
        	count_of_low_use_allocated_way0+= 1;
            functions_with_low_use.insert(current_function_callee_address); 
         }
         if (((!function_invocation_count[current_function_callee_address].low_degree_function) &&(function_invocation_count[current_function_callee_address].func_invocation_count>=INVOCATION_THRESHOLD)
        	&&(!temp.icache_hit)))
             count_misses_from_high_degree_functions_normal_cache++;
         else if (((function_invocation_count[current_function_callee_address].low_degree_function)&&(!temp.icache_hit))){
             count_misses_from_low_degree_functions_normal_cache++;
	     if (call_instr_seen)
		     count_missses_from_low_degree_functions_normal_cache_after_call++;
	 }
         if (!temp1.icache_hit){
    	     const ADDRINT notLineMask = ~(64 - 1);
             uint64_t current_cache_block_address = addr&notLineMask;
             //remove current block from the list of high use blocks, because we have already counted
             //its removal once and have also accounted for any cascade if any above. 
             list_of_high_use_blocks_replaced.erase(current_cache_block_address);	
         }
////	  
//////       }	
        if (!temp1.icache_hit){
           if (call_instr_seen){

        	if (function_invocation_count.find(current_function_callee_address) != function_invocation_count.end()){
        		function_invocation_count[current_function_callee_address].func_miss_count++;
        		function_invocation_count[current_function_callee_address].func_total_miss_count++;
        		function_invocation_count[current_function_callee_address].func_invocation_count++;
		}
	   }
           else
            function_invocation_count[current_function_callee_address].func_total_miss_count++;
        }
       else{
          if (call_instr_seen)     
           function_invocation_count[current_function_callee_address].func_invocation_count++;
       }
       call_instr_seen = false;
       ind_jump_seen = false;
       return_instr_seen = false;
       syscall_seen = false;
       dir_jump_instr_seen = false;
   }	
	
	
  // il1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);    
}


VOID Syscall_Instruction_Single(ADDRINT iaddr, THREADID tid)
{
    if (tid == _THREADID){
       LoadSingleFast(iaddr,tid);
       syscall_seen = true;
    }
}

VOID Direct_Call_Instruction_Single(ADDRINT iaddr, THREADID tid)
{
    if (tid == _THREADID){
       LoadSingleFast(iaddr,tid);
       call_instr_seen = true;
    }
}

VOID Direct_Jump_Instruction_Single(ADDRINT iaddr, THREADID tid)
{
    if (tid == _THREADID){
    	LoadSingleFast(iaddr,tid);
    	dir_jump_instr_seen = true;
    	prev_dir_jump_page = (iaddr/4096);
    }
}

VOID Indirect_Call_Instruction_Single(ADDRINT iaddr, THREADID tid)
{

    if (tid == _THREADID){
        LoadSingleFast(iaddr,tid);
        call_instr_seen = true;
        ind_call_instr_seen = true;
    }
}


VOID Indirect_Jump_Instruction_Single(ADDRINT iaddr, THREADID tid)
{

    if (tid == _THREADID){
        LoadSingleFast(iaddr,tid);
        ind_jump_seen = true;
        prev_ind_jump_page = iaddr/4096;
    }
}

VOID Return_Instruction_Single(ADDRINT iaddr, THREADID tid)
{

    if (tid == _THREADID){
         LoadSingleFast(iaddr,tid);
         //set that we have seen a return instruction
         //cleared at the end of processing the next instruction.
         return_instr_seen = true;
    }
}


VOID Syscall_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{
    if (tid == _THREADID){
       LoadSingleFast(iaddr,tid);
       syscall_seen = true;
    }
}

VOID Direct_Call_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{

    if (tid == _THREADID){
         LoadMultiFast(iaddr,size,tid);
         call_instr_seen = true;
    }
}

VOID Direct_Jump_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{
    if (tid == _THREADID){
         LoadMultiFast(iaddr,size,tid);
	 dir_jump_instr_seen = true;
	 prev_dir_jump_page = (iaddr/4096);
    }
}

VOID Indirect_Call_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{
    if (tid == _THREADID){
         LoadMultiFast(iaddr,size,tid);
         call_instr_seen = true;
         ind_call_instr_seen = true;
    }
}

VOID Indirect_Jump_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{
    if (tid == _THREADID){
         LoadMultiFast(iaddr,size,tid);
         ind_jump_seen = true;
         prev_ind_jump_page = iaddr/4096;
    }
}

VOID Return_Instruction_Multiple(ADDRINT iaddr, UINT32 size, THREADID tid)
{

    if (tid == _THREADID){
         LoadMultiFast(iaddr,size,tid);
        // mapping_from_page_to_unique_cache_blocks_touched_on_function_return[iaddr/4096].insert(iaddr/64);
         return_instr_seen = true;
    }
}

// The running count of instructions is kept here
// make it static to help the compiler optimize docount
static UINT64 icount = 0;

// This function is called before every instruction is executed
VOID docount(THREADID tid) { 
	
    if ((tid == _THREADID)){
#ifdef ACTIVE_LOW_FUNCTION_LOGGING 
	if ((icount%1000000) == 0){
	 uint64_t num_of_active_functions = 
		 number_of_active_low_use_functions.size();
	 list_of_active_low_use_function_counts.push_back(num_of_active_functions);	
	}
#endif	
	if (icount == INSTRUCTION_THRESHOLD){
         std::ofstream out(KnobOutputFile.Value().c_str());
     
         // print I-cache profile
         // @todo what does this print
         
         out << "PIN:MEMLATENCIES 1.0. 0x0\n";
                 
         out <<
             "#\n"
             "# ICACHE stats\n"
             "#\n";
         
         out << il1->StatsLong("# ", CACHE_BASE::CACHE_TYPE_ICACHE);
         out <<
             "#\n"
             "# ITLB stats\n"
             "#\n";
     
     
         out << itlb->StatsLong("# ", CACHE_BASE::CACHE_TYPE_ICACHE);
     
         if (KnobTrackInsts) {
             out <<
                 "#\n"
                 "# INST stats\n"
                 "#\n";
             
             out << profile.StringLong();
         }
        // out<<"ITLB misses from different miss categories" <<endl;
        // out<<"ITLB misses after call "<< itlb_misses_after_call <<endl;
         out <<"Total misses :" <<total_misses <<endl;
	 out <<"Misses from low degree of use functions (modifided cache): " << count_misses_from_low_degree_functions<< endl;
	 out <<"Misses from low degree of use functions (normal cache): " << count_misses_from_low_degree_functions_normal_cache<< endl;
	 out <<"Misses from low degree of use functions after call (modifided cache): " << count_missses_from_low_degree_functions_after_call<< endl;
	 out <<"Misses from low degree of use functions after call (normal cache): " << count_missses_from_low_degree_functions_normal_cache_after_call<< endl;
	 out <<"Misses from high degree of use functions (modifided cache): " << count_misses_from_high_degree_functions<< endl;
	 out <<"Misses from high degree of use functions (normal cache): " << count_misses_from_high_degree_functions_normal_cache<< endl;
	 out <<"Misses from medium degree of use functions (modifided cache): " << count_misses_from_medium_degree_functions<< endl;
	 out <<"Misses from medium degree of use functions (normal cache): " << count_misses_from_medium_degree_functions_normal_cache<< endl;
	 out <<"Cache blocks replaced from high use functions by high use functions: " << count_of_blocks_displaced_from_high_use_functions_by_high_use_functions << endl;
    	out <<"Cache blocks replaced from high use functions by low use (<=1) functions: " << count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions << endl;
    	out <<"Cache blocks replaced from high use functions by low use (<=2) functions: " << count_of_blocks_displaced_from_high_use_functions_by_low_use_two_functions << endl;
    	out <<"Cache blocks replaced from high use functions by low use (<=1) functions in cascade: " << 
		count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade << endl;
 	out <<"Cache blocks replaced from low use functions by low use functions: " <<
		count_of_low_use_displacing_low_use_functions <<endl;	
	out <<"Total number of low degree of use functions: " << functions_with_low_use.size() <<endl;
         out <<"Total number of functions: " << function_invocation_count.size() <<endl;
         out <<"Set of functions on which we have a icache miss after direct call" << endl;
         out <<"Number of low use functions allocated way0:" << count_of_low_use_allocated_way0 << endl;
         out <<"Number of low use functions we encounter a miss on:" << total_misses_on_low_use_functions << endl;
	 for(set<uint64_t>::const_iterator it = functions_with_low_use.begin();
             it != functions_with_low_use.end(); ++it)
         {
            out <<  (*it) << "," <<endl;
         }
         out << endl;

#ifdef PERLBENCH_DEBUG
	 out <<"Number of cache blocks accessed by function of interest in perlbench" << 
		number_of_cache_blocks_part_of_function_of_interest_perlbench.size() << endl; 
	 out << "Cache block accessed is: {";
	 for(set<uint64_t>::const_iterator it = number_of_cache_blocks_part_of_function_of_interest_perlbench.begin();
             it != number_of_cache_blocks_part_of_function_of_interest_perlbench.end(); ++it)
         {
	    out <<  (*it) << ",";
	 }
	 out << endl;
	 out <<"Number of instructions executed by function of interest in perlbench" << 
		instructions_spent_in_function_of_interest << endl; 
#endif	 
	 out <<"Active low use function counts at different instants {";
	 for(vector<uint64_t>::const_iterator it = list_of_active_low_use_function_counts.begin();
             it != list_of_active_low_use_function_counts.end(); ++it){
	    out <<  (*it) <<",";
	 }
	 out <<endl;
	 for(map<uint64_t,function_stats>::const_iterator it = function_invocation_count.begin();
             it != function_invocation_count.end(); ++it)
         {
             //print stats only for the pages that have more than a compulsory miss. 
                     out << "("<< (it->first) <<"): " << " number_of_times_function_is_missed: "<< function_invocation_count[it->first].func_miss_count <<" number_of_total_misses_from_function:  "<<function_invocation_count[it->first].func_total_miss_count  <<" number_of_times_function_is_invoked: " <<function_invocation_count[it->first].func_invocation_count<<" number_of_function_itlb_misses: "<< function_invocation_count[it->first].func_total_itlb_miss_count<< endl;
         }
     
         out<<"ICache misses from shared library "<< icache_misses_from_shared_library <<endl;
         out.close();
	 //special case SPEC programs were we sample the 0th thread. 
	 //exit(0);
	 //done = true;
        }
	icount++;
    }
 
}


/* ===================================================================== */

VOID Instruction(INS ins, void * v)
{
    
//    // Insert a call to docount before every instruction, no arguments are passed
    INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)docount, 
		    IARG_THREAD_ID, IARG_END);
    // map sparse INS addresses to dense IDs
    const ADDRINT iaddr = INS_Address(ins);
    const UINT32 instId = profile.Map(iaddr);

    const UINT32 size   = INS_Size(ins);
    const BOOL   single = (size <= 4);
                
    if (KnobTrackInsts) {
        if (single) {
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE, (AFUNPTR) LoadSingle,
                                     IARG_ADDRINT, iaddr,
                                     IARG_UINT32, instId,
                                     IARG_END);
        }
        else {
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE, (AFUNPTR) LoadMulti,
                                     IARG_UINT32, iaddr,
                                     IARG_UINT32, size,
                                     IARG_UINT32, instId,
                                     IARG_END);
        }
    }
    else {
        if (single) {
           if (INS_IsRet(ins)){
                 INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Return_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);
           }

           else if(INS_IsDirectControlFlow(ins) ){
             if( INS_IsCall(ins) )
                 INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Direct_Call_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);
             else
              INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Direct_Jump_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);
           }
           else if (INS_IsIndirectControlFlow(ins)){
             if( INS_IsCall(ins) )
                   INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Indirect_Call_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);

             else
              INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Indirect_Jump_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);
           }
           else if(INS_IsSyscall(ins))
           {
               INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Syscall_Instruction_Single, IARG_UINT32, iaddr,
                            IARG_THREAD_ID,  IARG_END);
           }
           else	       
            INS_InsertPredicatedCall(ins, IPOINT_BEFORE, (AFUNPTR) LoadSingleFast,
                                     IARG_UINT32, iaddr,
				     IARG_THREAD_ID,
                                     IARG_END);
        }
        else {
           if(INS_IsDirectControlFlow(ins) ){
             if( INS_IsCall(ins) )
                INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Direct_Call_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
             else
             INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Direct_Jump_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
           }
           else if (INS_IsIndirectControlFlow(ins)){
             if( INS_IsCall(ins) )
             INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Indirect_Call_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
             else
             INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Indirect_Jump_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
           }

           else if (INS_IsRet(ins)){
             INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Return_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
           }
           else if(INS_IsSyscall(ins))
           {
               INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Syscall_Instruction_Multiple, IARG_UINT32, iaddr,
                            IARG_UINT32, size,IARG_THREAD_ID,  IARG_END);
           }
           else            
		INS_InsertPredicatedCall(ins, IPOINT_BEFORE, (AFUNPTR) LoadMultiFast,
                                     IARG_UINT32, iaddr,
                                     IARG_UINT32, size,
				     IARG_THREAD_ID,
                                     IARG_END);
        }
    }
}

/* ===================================================================== */

VOID Fini(int code, VOID * v)
{

   // std::ofstream out(KnobOutputFile.Value().c_str());

   // // print I-cache profile
   // // @todo what does this print
   // 
   // out << "PIN:MEMLATENCIES 1.0. 0x0\n";
   //         
   // out <<
   //     "#\n"
   //     "# ICACHE stats\n"
   //     "#\n";
   // 
   // out << il1->StatsLong("# ", CACHE_BASE::CACHE_TYPE_ICACHE);

   // if (KnobTrackInsts) {
   //     out <<
   //         "#\n"
   //         "# INST stats\n"
   //         "#\n";
   //     
   //     out << profile.StringLong();
   // }
   // 
   // out <<
   //     "#\n"
   //     "# ITLB stats\n"
   //     "#\n";
   // 
   // 
   // out << itlb->StatsLong("# ", CACHE_BASE::CACHE_TYPE_ICACHE);
   // 
   // if (KnobTrackInsts) {
   //     out <<
   //         "#\n"
   //         "# INST stats\n"
   //         "#\n";
   //     
   //     out << profile.StringLong();
   // }
   //  out<<"ITLB misses from different miss categories" <<endl;
   //  out<<"ITLB misses after call "<< itlb_misses_after_call <<endl;
   // out <<"Total misses :" <<total_misses <<endl;
   // out <<"Misses from low degree of use functions (modifided cache): " << count_misses_from_low_degree_functions<< endl;
   // out <<"Misses from low degree of use functions (normal cache): " << count_misses_from_low_degree_functions_normal_cache<< endl;
   // out <<"Misses from low degree of use functions after call (modifided cache): " << count_missses_from_low_degree_functions_after_call<< endl;
   // out <<"Misses from low degree of use functions after call (normal cache): " << count_missses_from_low_degree_functions_normal_cache_after_call<< endl;
   // out <<"Misses from high degree of use functions (modifided cache): " << count_misses_from_high_degree_functions<< endl;
   // out <<"Misses from high degree of use functions (normal cache): " << count_misses_from_high_degree_functions_normal_cache<< endl;
   // out <<"Misses from medium degree of use functions (modifided cache): " << count_misses_from_medium_degree_functions<< endl;
   // out <<"Misses from medium degree of use functions (normal cache): " << count_misses_from_medium_degree_functions_normal_cache<< endl;
   // out <<"Cache blocks replaced from high use functions by high use functions: " << count_of_blocks_displaced_from_high_use_functions_by_high_use_functions << endl;
   // out <<"Cache blocks replaced from high use functions by low use (<=1) functions: " << count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions << endl;
   // out <<"Cache blocks replaced from high use functions by low use (<=2) functions: " << count_of_blocks_displaced_from_high_use_functions_by_low_use_two_functions << endl;
   // out <<"Cache blocks replaced from high use functions by low use (<=1) functions in cascade: " << 
   // 	count_of_blocks_displaced_from_high_use_functions_by_low_use_one_functions_in_cascade << endl;
   // out <<"Cache blocks replaced from low use functions by low use functions: " <<
   // 	count_of_low_use_displacing_low_use_functions <<endl;	
   // out <<"Total number of low degree of use functions: " << functions_with_low_use.size() <<endl;
   //  out <<"Total number of functions: " << function_invocation_count.size() <<endl;
   //  out <<"Set of functions on which we have a icache miss after direct call" << endl;
   //  out <<"Number of low use functions allocated way0:" << count_of_low_use_allocated_way0 << endl;
   //  out <<"Number of low use functions we encounter a miss on:" << total_misses_on_low_use_functions << endl;
   //  for(set<uint64_t>::const_iterator it = functions_with_low_use.begin();
   //      it != functions_with_low_use.end(); ++it)
   //  {
   //     out <<  (*it) << "," <<endl;
   //  }
   //  out << endl;
   // 
//#ifdef PERLBENCH_DEBUG
   //      out <<"Number of cache blocks accessed by function of interest in perlbench" << 
   //     	number_of_cache_blocks_part_of_function_of_interest_perlbench.size() << endl; 
   //      out << "Cache block accessed is: {";
   //      for(set<uint64_t>::const_iterator it = number_of_cache_blocks_part_of_function_of_interest_perlbench.begin();
   //          it != number_of_cache_blocks_part_of_function_of_interest_perlbench.end(); ++it)
   //      {
   //         out <<  (*it) << ",";
   //      }
   //      out << endl;
   //      out <<"Number of instructions executed by function of interest in perlbench" << 
   //     	instructions_spent_in_function_of_interest << endl; 
//#endif	 
   //      out <<"Active low use function counts at different instants {";
   //      for(vector<uint64_t>::const_iterator it = list_of_active_low_use_function_counts.begin();
   //          it != list_of_active_low_use_function_counts.end(); ++it){
   //         out <<  (*it) <<",";
   //      }
   //      out <<endl;
   //      uint64_t working_set_size = 0;
   //      for(map<uint64_t,function_stats>::const_iterator it = function_invocation_count.begin();
   //          it != function_invocation_count.end(); ++it)
   //      {
   //          //print stats only for the pages that have more than a compulsory miss. 
   //              if (function_invocation_count[it->first].func_invocation_count >= 10)
   //     	   working_set_size += function_invocation_count[it->first].unique_cache_blocks_touched_by_function.size();
   //     	 out << "("<< (it->first) <<"): " << " number_of_times_function_is_missed: "<< function_invocation_count[it->first].func_miss_count <<" number_of_total_misses_from_function:  "<<function_invocation_count[it->first].func_total_miss_count  <<" number_of_times_function_is_invoked: " <<function_invocation_count[it->first].func_invocation_count<<" number_of_function_itlb_misses: "<< function_invocation_count[it->first].func_total_itlb_miss_count<< endl;
   //      }
   //  	 out <<"Working set size is: " <<working_set_size << endl;
   //      out<<"ICache misses from shared library "<< icache_misses_from_shared_library <<endl;
   //      out.close();
    
}

/* ===================================================================== */

int main(int argc, char *argv[])
{
    PIN_InitSymbols();

    if( PIN_Init(argc,argv) )
    {
        return Usage();
    }

    il1 = new IL1::CACHE("L1 Inst Cache",
                         KnobCacheSize.Value() * KILO,
                         KnobLineSize.Value(),
                         KnobAssociativity.Value());

    itlb = new ITLB::CACHE("ITLB",
                         KnobITLBSize.Value() * KILO,
                         KnobITLBLineSize.Value(),
                         KnobITLBAssociativity.Value());
    profile.SetKeyName("iaddr          ");
    profile.SetCounterName("icache:miss        icache:hit");

    COUNTER_HIT_MISS threshold;

    threshold[COUNTER_HIT] = KnobThresholdHit.Value();
    threshold[COUNTER_MISS] = KnobThresholdMiss.Value();
    
    profile.SetThreshold( threshold );
    
    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    // Never returns

    PIN_StartProgram();
    
    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
