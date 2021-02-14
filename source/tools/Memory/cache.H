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
 *  This file contains a configurable cache class
 */

#ifndef PIN_CACHE_H
#define PIN_CACHE_H


#define KILO 1024
#define MEGA (KILO*KILO)
#define GIGA (KILO*MEGA)

#define EXTRA_WAYS 3

#define EXTRA_WAYS_1 3
typedef UINT64 CACHE_STATS; // type of cache hit/miss counters

#include <iostream>
#include <set>
#include <sstream>
using std::string;
using std::ostringstream;
using namespace std;

uint64_t total_accesses = 0;
#define VICTIM_CACHE_ADDITION 0
struct victim_buffer{
 uint64_t addr;
 bool valid;
 uint64_t timestamp;
};
#define NUM_VICTIM_ENTRIES 32
victim_buffer low_use_victim_entries[NUM_VICTIM_ENTRIES];

bool victim_buffer_entries_initialized = false;

uint64_t total_misses_on_low_use_function = 0;

/*! RMR (rodric@gmail.com) 
 *   - temporary work around because decstr()
 *     casts 64 bit ints to 32 bit ones
 */
static string mydecstr(UINT64 v, UINT32 w)
{
    ostringstream o;
    o.width(w);
    o << v;
    string str(o.str());
    return str;
}

/*!
 *  @brief Checks if n is a power of 2.
 *  @returns true if n is power of 2
 */
static inline bool IsPower2(UINT32 n)
{
    return ((n & (n - 1)) == 0);
}

struct use_and_blk_addr{
	bool function_use_information;
	uint64_t blk_addr;
	uint32_t allocated_way;
	uint32_t total_low_use_misses;
};

struct hit_and_use_information{
	bool icache_hit;
	bool function_use_information;
	//line addresses
	vector<uint64_t> blk_addresses;
	uint32_t allocated_way;
	uint32_t total_low_use_misses;
};

/*!
 *  @brief Computes floor(log2(n))
 *  Works by finding position of MSB set.
 *  @returns -1 if n == 0.
 */
static inline INT32 FloorLog2(UINT32 n)
{
    INT32 p = 0;

    if (n == 0) return -1;

    if (n & 0xffff0000) { p += 16; n >>= 16; }
    if (n & 0x0000ff00)	{ p +=  8; n >>=  8; }
    if (n & 0x000000f0) { p +=  4; n >>=  4; }
    if (n & 0x0000000c) { p +=  2; n >>=  2; }
    if (n & 0x00000002) { p +=  1; }

    return p;
}

/*!
 *  @brief Computes floor(log2(n))
 *  Works by finding position of MSB set.
 *  @returns -1 if n == 0.
 */
static inline INT32 CeilLog2(UINT32 n)
{
    return FloorLog2(n - 1) + 1;
}

/*!
 *  @brief Cache tag - self clearing on creation
 */
class CACHE_TAG
{
  private:
    ADDRINT _tag;

  public:
    CACHE_TAG(ADDRINT tag = 0) { _tag = tag; }
    bool operator==(const CACHE_TAG &right) const { return _tag == right._tag; }
    operator ADDRINT() const { return _tag; }
};


/*!
 * Everything related to cache sets
 */
namespace CACHE_SET
{

/*!
 *  @brief Cache set direct mapped
 */
class DIRECT_MAPPED
{
  private:
    CACHE_TAG _tag;

  public:
    DIRECT_MAPPED(UINT32 associativity = 1) { ASSERTX(associativity == 1); }

    VOID SetAssociativity(UINT32 associativity) { ASSERTX(associativity == 1); }
    UINT32 GetAssociativity(UINT32 associativity) { return 1; }

    UINT32 Find(CACHE_TAG tag) { return(_tag == tag); }
    UINT32 Find(CACHE_TAG tag, bool degree_of_use) { return(_tag == tag); }
    VOID Replace(CACHE_TAG tag) { _tag = tag; }
    use_and_blk_addr Replace_GetDegreeOfUse(CACHE_TAG tag, bool degree_of_use, uint64_t blk_addr) {
	    use_and_blk_addr temp;
	    temp.function_use_information = false;
	    temp.blk_addr = 0;
	    return temp;
    }
};

/*!
 *  @brief Cache set with round robin replacement
 */
template <UINT32 MAX_ASSOCIATIVITY = 4>
class ROUND_ROBIN
{
  private:
    CACHE_TAG _tags[MAX_ASSOCIATIVITY];
    //add the last access number as the proxy for last reference time
    //to enable LRU based replacement
    uint64_t _tag_last_reference_time[MAX_ASSOCIATIVITY];
    
    //track degree of use for each block (whether it comes from a function with high use or low use)
    //default set to low use.false indicates low use, true indicates high use.  
    bool _degree_of_use[MAX_ASSOCIATIVITY];
   

    //store the address of the item also alongside, so we can retrieve it on a replacement. 
    uint64_t _addr[MAX_ASSOCIATIVITY];
    
    UINT32 _tagsLastIndex;
    UINT32 _nextReplaceIndex;

  public:
    ROUND_ROBIN(UINT32 associativity = MAX_ASSOCIATIVITY)
      : _tagsLastIndex(associativity - 1)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _nextReplaceIndex = _tagsLastIndex;

        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
         _tags[index] = CACHE_TAG(0);
         _tag_last_reference_time[index] = 0;
	 _degree_of_use[index] = false;
	 _addr[index] = 0;
	}
    }

    VOID SetAssociativity(UINT32 associativity)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _tagsLastIndex = associativity - 1;
        _nextReplaceIndex = _tagsLastIndex;
    }
    UINT32 GetAssociativity(UINT32 associativity) { return _tagsLastIndex + 1; }
    
    UINT32 Find(CACHE_TAG tag)
    {
        bool result = true;
	total_accesses++;
        
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...

            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
        result = false;

        end: return result;
    }

    //functions may start with a low degree of use and then progress to have a
    // high degree of use. 
    UINT32 Find_UpdateDegreeOfUse(ADDRINT addr, CACHE_TAG tag, bool degree_of_use, bool medium_degree_of_use)
    {
        bool result = true;
	total_accesses++;
        
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...
            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    //update the degree of use. 
		    _degree_of_use[index] = degree_of_use;
		    goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
        result = false;
        end: return result;
    }
    VOID Replace(CACHE_TAG tag)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
    
        uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
          if(min_access_time>_tag_last_reference_time[index]){
               _nextReplaceIndex = index;
               min_access_time = _tag_last_reference_time[index];
            }
        }
	const UINT32 index = _nextReplaceIndex;
	
        _tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
    }

    use_and_blk_addr Replace_GetDegreeOfUse(CACHE_TAG tag, bool degree_of_use,uint64_t blk_addr, bool medium_degree_of_use)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
    
        uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        //way 0 is for low use functions and all other ways are for high use functions. 
	//see how this fares.
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
              if(min_access_time>_tag_last_reference_time[index]){
                 _nextReplaceIndex = index;
                 min_access_time = _tag_last_reference_time[index];
              }
        }
	const UINT32 index = _nextReplaceIndex;
	bool replaced_block_degree_of_use = _degree_of_use[index];
        uint64_t replaced_block_address = _addr[index];
	use_and_blk_addr temp;
	temp.function_use_information = replaced_block_degree_of_use;
	temp.blk_addr = replaced_block_address;
	_tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
	_degree_of_use[index] = degree_of_use;
	_addr[index] = blk_addr;
	return temp;
    }
};

template <UINT32 MAX_ASSOCIATIVITY = 4>
class MODIFIED_CACHE
{
  private:
    CACHE_TAG _tags[MAX_ASSOCIATIVITY];
    //add the last access number as the proxy for last reference time
    //to enable LRU based replacement
    uint64_t _tag_last_reference_time[MAX_ASSOCIATIVITY];
    
    //track degree of use for each block (whether it comes from a function with high use or low use)
    //default set to low use.false indicates low use, true indicates high use.  
    bool _degree_of_use[MAX_ASSOCIATIVITY];
   
    //bit is set for low use functions whose degree of use is greater than one. 
    bool _medium_degree_of_use[MAX_ASSOCIATIVITY];

    //store the address of the item also alongside, so we can retrieve it on a replacement. 
    uint64_t _addr[MAX_ASSOCIATIVITY];
    
    UINT32 _tagsLastIndex;
    UINT32 _nextReplaceIndex;

  public:
    MODIFIED_CACHE(UINT32 associativity = MAX_ASSOCIATIVITY)
      : _tagsLastIndex(associativity - 1)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _nextReplaceIndex = _tagsLastIndex;

        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
         _tags[index] = CACHE_TAG(0);
         _tag_last_reference_time[index] = 0;
	 _degree_of_use[index] = false;
	 _medium_degree_of_use[index] = false;
	 _addr[index] = 0;
	}
	if (!victim_buffer_entries_initialized){
	   for (INT32 index = (NUM_VICTIM_ENTRIES-1); 
	      	  index >=0; index--)
	  {
	    low_use_victim_entries[_nextReplaceIndex].valid = false;
	    low_use_victim_entries[_nextReplaceIndex].addr = 0;
	    low_use_victim_entries[_nextReplaceIndex].timestamp = 0;
	  }
	  victim_buffer_entries_initialized = true;
	}
    }

    VOID SetAssociativity(UINT32 associativity)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _tagsLastIndex = associativity - 1;
        _nextReplaceIndex = _tagsLastIndex;
    }
    UINT32 GetAssociativity(UINT32 associativity) { return _tagsLastIndex + 1; }
    
    UINT32 Find(CACHE_TAG tag)
    {
        bool result = true;
	total_accesses++;
        
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...

            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
        result = false;

        end: return result;
    }

    //functions may start with a low degree of use and then progress to have a
    // high degree of use. 
    UINT32 Find_UpdateDegreeOfUse(ADDRINT addr, CACHE_TAG tag, bool degree_of_use, bool medium_degree_of_use)
    {
        bool result = true;
	total_accesses++;
        bool found = false; 
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...

            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    
	   	   //Retain at LRU position when the degree of use of function is 
	   	   //low. 
	   	   if (!degree_of_use)
	   	  	 _tag_last_reference_time[index] = 0;
		    //update the degree of use. 
		    _degree_of_use[index] = degree_of_use;
		    found = true;
		   // goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
	//generate block address and compare against
	//entries in the victim buffer.
#ifdef VICTIM_CACHE_ADDITION
	ADDRINT block_address = addr/64;
        //check performed only for medium degree of use functions.
	if ((!found) && (medium_degree_of_use)){
	  for (int i = 0;i<NUM_VICTIM_ENTRIES;i++){
	     if (low_use_victim_entries[i].valid){
	       if (low_use_victim_entries[i].addr == 
			       block_address){
	           found = true;
		   //update the timstamp so that stale low use blocks that are 
		   //kicked out can get thrown out of the cache
		   //sooner. 
		   low_use_victim_entries[i].timestamp = total_accesses; 
		}
	     }
	   }
	}
#endif
	if (!found)	
           result = false;
         return result;
    }
    VOID Replace(CACHE_TAG tag)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
    
        uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
          if(min_access_time>_tag_last_reference_time[index]){
               _nextReplaceIndex = index;
               min_access_time = _tag_last_reference_time[index];
            }
        }
	const UINT32 index = _nextReplaceIndex;
	
        _tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
    }

    use_and_blk_addr Replace_GetDegreeOfUse(CACHE_TAG tag, bool degree_of_use,uint64_t blk_addr, bool medium_degree_of_use)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
	
	uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        //way 0 is for low use functions and all other ways are for high use functions. 
	//see how this fares.
//	if (degree_of_use){
//		for (INT32 index = _tagsLastIndex; index > (EXTRA_WAYS-1); index--)
//        	{
//          	   if(min_access_time>_tag_last_reference_time[index]){
//               	      _nextReplaceIndex = index;
//               	      min_access_time = _tag_last_reference_time[index];
//            	   }
//        	}
//	}
//	else{
//		min_access_time = _tag_last_reference_time[(EXTRA_WAYS-1)];
//		_nextReplaceIndex = (EXTRA_WAYS-1);
//		for (INT32 index = (EXTRA_WAYS-1); index >= 0; index--)
//        	{
//          	   if(min_access_time>_tag_last_reference_time[index]){
//               	      _nextReplaceIndex = index;
//               	      min_access_time = _tag_last_reference_time[index];
//            	   }
//        	}
//	   // _nextReplaceIndex = 0;
//	}
	
	for (INT32 index = _tagsLastIndex; index >=0; index--)
        {
           if(min_access_time>_tag_last_reference_time[index]){
              _nextReplaceIndex = index;
              min_access_time = _tag_last_reference_time[index];
           }
        }
	if (((_nextReplaceIndex == 0)||(_nextReplaceIndex == 1)) && (!degree_of_use))
		total_misses_on_low_use_function++;
	const UINT32 index = _nextReplaceIndex;
	bool replaced_block_degree_of_use = _degree_of_use[index];
        uint64_t replaced_block_address = _addr[index];
	use_and_blk_addr temp;
	temp.function_use_information = replaced_block_degree_of_use;
        bool medium_block_degree_of_use = _medium_degree_of_use[index]; 	
	 temp.blk_addr = replaced_block_address;
	//if replaced block is from a low use function(deg of use > 1),
	//then find a spot to place in the victim buffer and 
	//place this block there. 
	if (medium_block_degree_of_use){
	  uint64_t block_address = temp.blk_addr/64;
	  uint64_t min_access_time = low_use_victim_entries[NUM_VICTIM_ENTRIES-1].timestamp; 
	  uint64_t _nextReplaceIndex = (NUM_VICTIM_ENTRIES-1);
	  for (INT32 index = (NUM_VICTIM_ENTRIES-1); 
			  index >=0; index--)
	  {
	     if(min_access_time>low_use_victim_entries[index].timestamp){
	        _nextReplaceIndex = index;
	        min_access_time = _tag_last_reference_time[index];
	     }
	  }
	  low_use_victim_entries[_nextReplaceIndex].valid = true;
	  low_use_victim_entries[_nextReplaceIndex].addr = block_address;
	  //placed at MRU position for now.
	  low_use_victim_entries[_nextReplaceIndex].timestamp = 0;
	  
	}
	temp.allocated_way = index;
	_tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
	//insert at LRU position when the degree of use of function is 
	//low. 
	if (!degree_of_use)
	        _tag_last_reference_time[index] = 0;
	_degree_of_use[index] = degree_of_use;
	_medium_degree_of_use[index] = medium_degree_of_use;
	_addr[index] = blk_addr;
	return temp;
    }
};

template <UINT32 MAX_ASSOCIATIVITY = 4>
class MODIFIED_CACHE_2
{
  private:
    CACHE_TAG _tags[MAX_ASSOCIATIVITY];
    //add the last access number as the proxy for last reference time
    //to enable LRU based replacement
    uint64_t _tag_last_reference_time[MAX_ASSOCIATIVITY];
    
    //track degree of use for each block (whether it comes from a function with high use or low use)
    //default set to low use.false indicates low use, true indicates high use.  
    bool _degree_of_use[MAX_ASSOCIATIVITY];
    
    //store the address of the item also alongside, so we can retrieve it on a replacement. 
    uint64_t _addr[MAX_ASSOCIATIVITY];
    
    UINT32 _tagsLastIndex;
    UINT32 _nextReplaceIndex;

  public:
    MODIFIED_CACHE_2(UINT32 associativity = MAX_ASSOCIATIVITY)
      : _tagsLastIndex(associativity - 1)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _nextReplaceIndex = _tagsLastIndex;

        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
         _tags[index] = CACHE_TAG(0);
         _tag_last_reference_time[index] = 0;
	 _degree_of_use[index] = false;
	 _addr[index] = 0;
	}
    }

    VOID SetAssociativity(UINT32 associativity)
    {
        ASSERTX(associativity <= MAX_ASSOCIATIVITY);
        _tagsLastIndex = associativity - 1;
        _nextReplaceIndex = _tagsLastIndex;
    }
    UINT32 GetAssociativity(UINT32 associativity) { return _tagsLastIndex + 1; }
    
    UINT32 Find(CACHE_TAG tag)
    {
        bool result = true;
	total_accesses++;
        
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...

            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
        result = false;

        end: return result;
    }

    //functions may start with a low degree of use and then progress to have a
    // high degree of use. 
    UINT32 Find_UpdateDegreeOfUse(ADDRINT addr, CACHE_TAG tag, bool degree_of_use, bool medium_degree_of_use)
    {
        bool result = true;
	total_accesses++;
        
	for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
            // this is an ugly micro-optimization, but it does cause a
            // tighter assembly loop for ARM that way ...

            if(_tags[index] == tag) {
		    _tag_last_reference_time[index] = total_accesses;
		    //update the degree of use. 
		    _degree_of_use[index] = degree_of_use;
		    goto end;
	    }
//		if(_tags[index] == tag) goto end;
        }
        result = false;

        end: return result;
    }
    VOID Replace(CACHE_TAG tag)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
    
        uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        for (INT32 index = _tagsLastIndex; index >= 0; index--)
        {
          if(min_access_time>_tag_last_reference_time[index]){
               _nextReplaceIndex = index;
               min_access_time = _tag_last_reference_time[index];
            }
        }
	const UINT32 index = _nextReplaceIndex;
	
        _tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
    }

    use_and_blk_addr Replace_GetDegreeOfUse(CACHE_TAG tag, bool degree_of_use,uint64_t blk_addr, bool medium_degree_of_use)
    {
        // g++ -O3 too dumb to do CSE on following lines?!
      //  const UINT32 index = _nextReplaceIndex;

      //  _tags[index] = tag;
      //  // condition typically faster than modulo
      //  _nextReplaceIndex = (index == 0 ? _tagsLastIndex : index - 1);
	
	uint64_t min_access_time = _tag_last_reference_time[_tagsLastIndex];
        uint64_t _nextReplaceIndex = _tagsLastIndex;
        //way 0 is for low use functions and all other ways are for high use functions. 
	//see how this fares.
	if (degree_of_use){
		for (INT32 index = _tagsLastIndex; index > (EXTRA_WAYS_1-1); index--)
        	{
          	   if(min_access_time>_tag_last_reference_time[index]){
               	      _nextReplaceIndex = index;
               	      min_access_time = _tag_last_reference_time[index];
            	   }
        	}
	}
	else{
		min_access_time = _tag_last_reference_time[(EXTRA_WAYS_1-1)];
		_nextReplaceIndex = (EXTRA_WAYS_1-1);
		for (INT32 index = (EXTRA_WAYS_1-1); index >= 0; index--)
        	{
          	   if(min_access_time>_tag_last_reference_time[index]){
               	      _nextReplaceIndex = index;
               	      min_access_time = _tag_last_reference_time[index];
            	   }
        	}
	   // _nextReplaceIndex = 0;
	}
	if (((_nextReplaceIndex == 0)||(_nextReplaceIndex == 1)) && (!degree_of_use))
		total_misses_on_low_use_function++;
	const UINT32 index = _nextReplaceIndex;
	bool replaced_block_degree_of_use = _degree_of_use[index];
        uint64_t replaced_block_address = _addr[index];
	use_and_blk_addr temp;
	temp.function_use_information = replaced_block_degree_of_use;
	temp.blk_addr = replaced_block_address;
	temp.allocated_way = index;
	_tags[index] = tag;
	_tag_last_reference_time[index] = total_accesses;
	_degree_of_use[index] = degree_of_use;
	_addr[index] = blk_addr;
	return temp;
    }
};

} // namespace CACHE_SET

namespace CACHE_ALLOC
{
    typedef enum 
    {
        STORE_ALLOCATE,
        STORE_NO_ALLOCATE
    } STORE_ALLOCATION;
}

/*!
 *  @brief Generic cache base class; no allocate specialization, no cache set specialization
 */
class CACHE_BASE
{
  public:
    // types, constants
    typedef enum 
    {
        ACCESS_TYPE_LOAD,
        ACCESS_TYPE_STORE,
        ACCESS_TYPE_NUM
    } ACCESS_TYPE;

    typedef enum
    {
        CACHE_TYPE_ICACHE,
        CACHE_TYPE_DCACHE,
        CACHE_TYPE_NUM
    } CACHE_TYPE;

  protected:
    static const UINT32 HIT_MISS_NUM = 2;
    CACHE_STATS _access[ACCESS_TYPE_NUM][HIT_MISS_NUM];

  private:    // input params
    const std::string _name;
    const UINT32 _cacheSize;
    const UINT32 _lineSize;
    const UINT32 _associativity;

    // computed params
    const UINT32 _lineShift;
    const UINT32 _setIndexMask;

    CACHE_STATS SumAccess(bool hit) const
    {
        CACHE_STATS sum = 0;

        for (UINT32 accessType = 0; accessType < ACCESS_TYPE_NUM; accessType++)
        {
            sum += _access[accessType][hit];
        }

        return sum;
    }

  protected:
    UINT32 NumSets() const { return _setIndexMask + 1; }

  public:
    // constructors/destructors
    CACHE_BASE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity);

    // accessors
    UINT32 CacheSize() const { return _cacheSize; }
    UINT32 LineSize() const { return _lineSize; }
    UINT32 Associativity() const { return _associativity; }
    //
    CACHE_STATS Hits(ACCESS_TYPE accessType) const { return _access[accessType][true];}
    CACHE_STATS Misses(ACCESS_TYPE accessType) const { return _access[accessType][false];}
    CACHE_STATS Accesses(ACCESS_TYPE accessType) const { return Hits(accessType) + Misses(accessType);}
    CACHE_STATS Hits() const { return SumAccess(true);}
    CACHE_STATS Misses() const { return SumAccess(false);}
    CACHE_STATS Accesses() const { return Hits() + Misses();}

    VOID SplitAddress(const ADDRINT addr, CACHE_TAG & tag, UINT32 & setIndex) const
    {
        tag = addr >> _lineShift;
        setIndex = tag & _setIndexMask;
    }

    VOID SplitAddress(const ADDRINT addr, CACHE_TAG & tag, UINT32 & setIndex, UINT32 & lineIndex) const
    {
        const UINT32 lineMask = _lineSize - 1;
        lineIndex = addr & lineMask;
        SplitAddress(addr, tag, setIndex);
    }


    VOID SpecialSplitAddress(const ADDRINT addr, CACHE_TAG & tag, UINT32 & setIndex) const
    {
        tag = addr >> _lineShift;
        setIndex = tag & _setIndexMask;
	uint64_t extra_tag_mask = tag >> _lineShift;
	uint64_t extra_index_to_xor_with = extra_tag_mask&_setIndexMask;
	setIndex = setIndex^extra_index_to_xor_with;	
    }

    VOID SpecialSplitAddress(const ADDRINT addr, CACHE_TAG & tag, UINT32 & setIndex, UINT32 & lineIndex) const
    {
        const UINT32 lineMask = _lineSize - 1;
        lineIndex = addr & lineMask;
        SpecialSplitAddress(addr, tag, setIndex);
    }
    string StatsLong(string prefix = "", CACHE_TYPE = CACHE_TYPE_DCACHE) const;
};

CACHE_BASE::CACHE_BASE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity)
  : _name(name),
    _cacheSize(cacheSize),
    _lineSize(lineSize),
    _associativity(associativity),
    _lineShift(FloorLog2(lineSize)),
    _setIndexMask((cacheSize / (associativity * lineSize)) - 1)
{

    ASSERTX(IsPower2(_lineSize));
    ASSERTX(IsPower2(_setIndexMask + 1));

    for (UINT32 accessType = 0; accessType < ACCESS_TYPE_NUM; accessType++)
    {
        _access[accessType][false] = 0;
        _access[accessType][true] = 0;
    }
}

/*!
 *  @brief Stats output method
 */

string CACHE_BASE::StatsLong(string prefix, CACHE_TYPE cache_type) const
{
    const UINT32 headerWidth = 19;
    const UINT32 numberWidth = 12;

    string out;
    
    out += prefix + _name + ":" + "\n";

    if (cache_type != CACHE_TYPE_ICACHE) {
       for (UINT32 i = 0; i < ACCESS_TYPE_NUM; i++)
       {
           const ACCESS_TYPE accessType = ACCESS_TYPE(i);

           std::string type(accessType == ACCESS_TYPE_LOAD ? "Load" : "Store");

           out += prefix + ljstr(type + "-Hits:      ", headerWidth)
                  + mydecstr(Hits(accessType), numberWidth)  +
                  "  " +fltstr(100.0 * Hits(accessType) / Accesses(accessType), 2, 6) + "%\n";

           out += prefix + ljstr(type + "-Misses:    ", headerWidth)
                  + mydecstr(Misses(accessType), numberWidth) +
                  "  " +fltstr(100.0 * Misses(accessType) / Accesses(accessType), 2, 6) + "%\n";
        
           out += prefix + ljstr(type + "-Accesses:  ", headerWidth)
                  + mydecstr(Accesses(accessType), numberWidth) +
                  "  " +fltstr(100.0 * Accesses(accessType) / Accesses(accessType), 2, 6) + "%\n";
        
           out += prefix + "\n";
       }
    }

    out += prefix + ljstr("Total-Hits:      ", headerWidth)
           + mydecstr(Hits(), numberWidth) +
           "  " +fltstr(100.0 * Hits() / Accesses(), 2, 6) + "%\n";

    out += prefix + ljstr("Total-Misses:    ", headerWidth)
           + mydecstr(Misses(), numberWidth) +
           "  " +fltstr(100.0 * Misses() / Accesses(), 2, 6) + "%\n";

    out += prefix + ljstr("Total-Accesses:  ", headerWidth)
           + mydecstr(Accesses(), numberWidth) +
           "  " +fltstr(100.0 * Accesses() / Accesses(), 2, 6) + "%\n";
    
    out += prefix + ljstr("Total-Low use misses:  ", headerWidth)
           + mydecstr(total_misses_on_low_use_function, numberWidth) +
           "%\n";
    out += "\n";

    return out;
}


/*!
 *  @brief Templated cache class with specific cache set allocation policies
 *
 *  All that remains to be done here is allocate and deallocate the right
 *  type of cache sets.
 */
template <class SET, UINT32 MAX_SETS, UINT32 STORE_ALLOCATION>
class CACHE : public CACHE_BASE
{
  private:
    SET _sets[MAX_SETS];
    
  public:
    // constructors/destructors
    CACHE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity)
      : CACHE_BASE(name, cacheSize, lineSize, associativity)
    {
        ASSERTX(NumSets() <= MAX_SETS);

        for (UINT32 i = 0; i < NumSets(); i++)
        {
            _sets[i].SetAssociativity(associativity);
        }
    }

    // modifiers
    /// Cache access from addr to addr+size-1
    bool Access(ADDRINT addr, UINT32 size, ACCESS_TYPE accessType);
    //smurthy
    //selectively allocate a line based on a allocate condition
    hit_and_use_information Access_selective_allocate(ADDRINT addr, UINT32 size, ACCESS_TYPE accessType, bool allocate, bool degree_of_use,bool medium_degree_of_use, bool special_cache_type);
    /// Cache access at addr that does not span cache lines
    bool AccessSingleLine(ADDRINT addr, ACCESS_TYPE accessType);
    //smurthy
    //selectively allocate a line based on a allocate condition
    hit_and_use_information AccessSingleLine_selective_allocate(ADDRINT addr, ACCESS_TYPE accessType, bool allocate, bool degree_of_use, bool medium_degree_of_use, bool special_cache_type);
};

/*!
 *  @return true if all accessed cache lines hit
 */

template <class SET, UINT32 MAX_SETS, UINT32 STORE_ALLOCATION>
bool CACHE<SET,MAX_SETS,STORE_ALLOCATION>::Access(ADDRINT addr, UINT32 size, ACCESS_TYPE accessType)
{
    const ADDRINT highAddr = addr + size;
    bool allHit = true;

    const ADDRINT lineSize = LineSize();
    const ADDRINT notLineMask = ~(lineSize - 1);
    do
    {
        CACHE_TAG tag;
        UINT32 setIndex;

        SplitAddress(addr, tag, setIndex);

        SET & set = _sets[setIndex];

        bool localHit = set.Find(tag);
        allHit &= localHit;

        // on miss, loads always allocate, stores optionally
        if ( (! localHit) && (accessType == ACCESS_TYPE_LOAD || STORE_ALLOCATION == CACHE_ALLOC::STORE_ALLOCATE))
        {
            set.Replace(tag);
        }

        addr = (addr & notLineMask) + lineSize; // start of next cache line
    }
    while (addr < highAddr);

    _access[accessType][allHit]++;

    return allHit;
}


template <class SET, UINT32 MAX_SETS, UINT32 STORE_ALLOCATION>
hit_and_use_information CACHE<SET,MAX_SETS,STORE_ALLOCATION>::Access_selective_allocate(ADDRINT addr, UINT32 size, ACCESS_TYPE accessType, bool selective_allocate, bool degree_of_use, bool medium_degree_of_use,  bool special_cache_type)
{
    const ADDRINT highAddr = addr + size;
    bool allHit = true;

    const ADDRINT lineSize = LineSize();
    const ADDRINT notLineMask = ~(lineSize - 1);
    hit_and_use_information temp;
    temp.icache_hit = false;
    temp.function_use_information = false;
    use_and_blk_addr temp1;
    do
    {
        CACHE_TAG tag;
        UINT32 setIndex;


        //SET & set;
       //	= _sets[setIndex];
        //bool localHit;
       //	= set.Find_UpdateDegreeOfUse(tag, degree_of_use);
        //search for low use function in the alternate location too, in case it is not found in the 
	//original location. Here on we allocate this block only in the alternate location. 
	if ((!degree_of_use) && (special_cache_type)){
        	SplitAddress(addr, tag, setIndex);
	}
	else{
        	SplitAddress(addr, tag, setIndex);
	}
	
        SET &set = _sets[setIndex];
        bool localHit = set.Find_UpdateDegreeOfUse(addr, tag, degree_of_use, medium_degree_of_use);
	allHit &= localHit;
        // on miss, loads always allocate, stores optionally
        if ((selective_allocate) && (!localHit) && (accessType == ACCESS_TYPE_LOAD || STORE_ALLOCATION == CACHE_ALLOC::STORE_ALLOCATE))
        {
           
	   temp1 = set.Replace_GetDegreeOfUse(tag, degree_of_use,addr&notLineMask, medium_degree_of_use);
	   temp.function_use_information |=temp1.function_use_information;
	   if (temp1.function_use_information)
	   	temp.blk_addresses.push_back(temp1.blk_addr);
	   temp.allocated_way = temp1.allocated_way;
        }

        addr = (addr & notLineMask) + lineSize; // start of next cache line
    }
    while (addr < highAddr);

    _access[accessType][allHit]++;
    
    temp.icache_hit = allHit;
    temp.total_low_use_misses = total_misses_on_low_use_function;
    return temp;
}

/*!
 *  @return true if accessed cache line hits
 */
template <class SET, UINT32 MAX_SETS, UINT32 STORE_ALLOCATION>
bool CACHE<SET,MAX_SETS,STORE_ALLOCATION>::AccessSingleLine(ADDRINT addr, ACCESS_TYPE accessType)
{
    CACHE_TAG tag;
    UINT32 setIndex;

    SplitAddress(addr, tag, setIndex);

    SET & set = _sets[setIndex];

    bool hit = set.Find(tag);

    // on miss, loads always allocate, stores optionally
    if ( (! hit) && (accessType == ACCESS_TYPE_LOAD || STORE_ALLOCATION == CACHE_ALLOC::STORE_ALLOCATE))
    {
        set.Replace(tag);
    }

    _access[accessType][hit]++;

    return hit;
}


template <class SET, UINT32 MAX_SETS, UINT32 STORE_ALLOCATION>
hit_and_use_information CACHE<SET,MAX_SETS,STORE_ALLOCATION>::AccessSingleLine_selective_allocate(ADDRINT addr, ACCESS_TYPE accessType, bool selective_allocate, 
						bool degree_of_use, bool medium_degree_of_use,  bool special_cache_type)
{
    CACHE_TAG tag;
    UINT32 setIndex;


    //SET & set; 

    const ADDRINT lineSize = LineSize();
    const ADDRINT notLineMask = ~(lineSize - 1);
    //bool hit;
   // = set.Find_UpdateDegreeOfUse(tag, degree_of_use);
    //search for low use function in the alternate location too, in case it is not found in the 
    //original location. Here on we allocate this block only in the alternate location. 
    if ((!degree_of_use) && (special_cache_type)){
    	SplitAddress(addr, tag, setIndex);
    }
    else {
    	SplitAddress(addr, tag, setIndex);
    }

    SET &set = _sets[setIndex];
    bool hit = set.Find_UpdateDegreeOfUse(addr, tag, degree_of_use, medium_degree_of_use);
    hit_and_use_information temp;
    use_and_blk_addr temp1;
    temp.icache_hit = hit;
    //by default return low use
    temp.function_use_information = false;

    // on miss, loads always allocate, stores optionally
    if ((selective_allocate)&& (! hit) && (accessType == ACCESS_TYPE_LOAD || STORE_ALLOCATION == CACHE_ALLOC::STORE_ALLOCATE))
    {
        
	temp1 = set.Replace_GetDegreeOfUse(tag, degree_of_use,addr&notLineMask, medium_degree_of_use);
	temp.function_use_information = temp1.function_use_information;
	//Add the cacheblocks to the vector only for high use functions, because
	//we are interested in the block addresses only for high use functions. 
	if (temp1.function_use_information)
	    temp.blk_addresses.push_back(temp1.blk_addr);
	temp.allocated_way = temp1.allocated_way;
    }

    _access[accessType][hit]++;
    temp.total_low_use_misses = total_misses_on_low_use_function;
    return temp;
}


// define shortcuts
#define CACHE_DIRECT_MAPPED(MAX_SETS, ALLOCATION) CACHE<CACHE_SET::DIRECT_MAPPED, MAX_SETS, ALLOCATION>
#define CACHE_ROUND_ROBIN(MAX_SETS, MAX_ASSOCIATIVITY, ALLOCATION) CACHE<CACHE_SET::ROUND_ROBIN<MAX_ASSOCIATIVITY>, MAX_SETS, ALLOCATION>
#define CACHE_MODIFIED_CACHE(MAX_SETS, MAX_ASSOCIATIVITY, ALLOCATION) CACHE<CACHE_SET::MODIFIED_CACHE<MAX_ASSOCIATIVITY>, MAX_SETS, ALLOCATION>
#define CACHE_MODIFIED_CACHE_2(MAX_SETS, MAX_ASSOCIATIVITY, ALLOCATION) CACHE<CACHE_SET::MODIFIED_CACHE_2<MAX_ASSOCIATIVITY>, MAX_SETS, ALLOCATION>
#endif // PIN_CACHE_H
