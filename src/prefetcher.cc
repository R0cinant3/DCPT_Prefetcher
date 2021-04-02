#include <iostream>
#include <vector>
#include <deque>
#include <iterator>
#include <algorithm>

#include "interface.hh"

#define ENTRY_BUFFER_LIMIT 100
#define FLIGHT_BUFFER_LIMIT 32
#define DELTA_BUFFER_LIMIT 6
#define DELTA_NUMBER_BITS 10
#define DELTA_MAX_VALUE ((1 << (DELTA_NUMBER_BITS - 1)) - 1)

struct DCPT_Entry
{
    //Constructor to initialize Last_Address and Last_Prefetch
    DCPT_Entry(Addr pc) : Last_Address(0), Last_Prefetch(0), pc(0){}
    DCPT_Entry()        : Last_Address(0), Last_Prefetch(0), pc(0){}
    Addr Last_Address, Last_Prefetch, pc;
    std::deque<Addr> deltas;
};

static DCPT_Entry *entry;
static std::deque<Addr> inFlight;
static std::vector<DCPT_Entry *> List_of_Entries;

static std::vector<Addr> Delta_Correlation(DCPT_Entry *);
static std::vector<Addr> Prefetch_Filtering(DCPT_Entry *, std::vector<Addr>);
static bool Delta_Address_Calculation_is_Nonzero(AccessStat);
static void Delta_Address_Store_in_Buffer(AccessStat, Addr);
static DCPT_Entry* Create_New_Entry(AccessStat);
static void Store_New_Entry(DCPT_Entry *);

static DCPT_Entry* Table_Look_Up(Addr pc){
    std::vector<DCPT_Entry *>::iterator it_List_of_Entries = List_of_Entries.begin();
    for(; it_List_of_Entries != List_of_Entries.end(); it_List_of_Entries++)
    {
        DCPT_Entry *Find_Entry = *it_List_of_Entries;
        if(pc == Find_Entry->pc)
            return Find_Entry;
    }
    return NULL;
}

void prefetch_init(void)
{
    /* Called before any calls to prefetch_access. */
    /* This is the place to initialize data structures. */
    entry = new DCPT_Entry;
    List_of_Entries.clear();
    inFlight.clear();
    DPRINTF(HWPrefetch, "Initialized sequential-on-access prefetcher\n");
}

void prefetch_access(AccessStat stat)
{
    static std::vector<Addr> Candidates;
    static std::vector<Addr> Prefetches;
    
    entry = Table_Look_Up(stat.pc);

    if(entry == NULL)
    {
        entry = Create_New_Entry(stat);
        Store_New_Entry(entry);
    }
    else if(Delta_Address_Calculation_is_Nonzero(stat)){
        Candidates = Delta_Correlation(entry);
        Prefetches = Prefetch_Filtering(entry, Candidates);

        std::vector<Addr>::iterator it_Prefetch = Prefetches.begin();
        for(; it_Prefetch != Prefetches.end(); it_Prefetch++)     
        {
            issue_prefetch(*it_Prefetch);
        }
    }
}

void prefetch_complete(Addr addr) 
{
    /*
     * Called when a block requested by the prefetcher has been loaded.
     */

    std::deque<Addr>::iterator it_inFlight = std::find(inFlight.begin(), inFlight.end(), addr);
    if(it_inFlight != inFlight.end())
        inFlight.erase(it_inFlight); 
    
}

//Delta Correlation returns Candidates, a circular buffer containing addresses.
static std::vector<Addr> Delta_Correlation(DCPT_Entry *entry)
{
    std::vector<Addr> Candidates;

    Addr delta_pair_last1 = *(entry->deltas.end() - 1);
    Addr delta_pair_last2 = *(entry->deltas.end() - 2);   
    Addr address = entry->Last_Address;

    std::deque<Addr>::iterator it_Deltas = entry->deltas.begin();
    //Iterate through each u, v pair in entry->Deltas
    while(it_Deltas != entry->deltas.end())
    {
        Addr delta_pair_1 = *it_Deltas; 
        Addr delta_pair_2 = *(++it_Deltas);
        //Check if u = d2 and v = d1
        if(delta_pair_1 == delta_pair_last2 && delta_pair_2 == delta_pair_last1)
        {   
            //Add the remaining deltas in entry->Deltas
            for(it_Deltas++; it_Deltas != entry->deltas.end(); it_Deltas++)
            {
                address += *it_Deltas * BLOCK_SIZE;
                Candidates.push_back(address);
            }
        }
    }   
    return Candidates;
}

static std::vector<Addr> Prefetch_Filtering(DCPT_Entry *entry, std::vector<Addr> Candidates)
{
    //In_Flight is a circular buffer
    //Prefetches is a circular buffer 
    std::vector<Addr> Prefetches;

    std::vector<Addr>::iterator it_Candidate = Candidates.begin();
    for(; it_Candidate != Candidates.end(); it_Candidate++)
    {
        if((std::find(inFlight.begin(), inFlight.end(), *it_Candidate) == inFlight.end()) && !in_cache(*it_Candidate) && !in_mshr_queue(*it_Candidate))
        {//Candidate is not in MSHR, Cache, or "other prefetch requests that are
        //not completed" buffer(inFlight)
            Prefetches.push_back(*it_Candidate);
            entry->Last_Prefetch = *it_Candidate;
            if(inFlight.size() == FLIGHT_BUFFER_LIMIT)
                inFlight.pop_front();
            inFlight.push_back(*it_Candidate);
        }
    }
    return Prefetches;
}

static bool Delta_Address_Calculation_is_Nonzero(AccessStat stat)
{
    Addr Delta_Address = stat.mem_addr - entry->Last_Address;
    Delta_Address /= BLOCK_SIZE >> 1;
    if(Delta_Address != 0)
    {
        Delta_Address_Store_in_Buffer(stat, Delta_Address);
        return true;
    }
    else
        return false;
}

static void Delta_Address_Store_in_Buffer(AccessStat stat, Addr Delta_Address)
{
    if(Delta_Address > DELTA_MAX_VALUE)
        Delta_Address = 0;

    if(entry->deltas.size() == DELTA_BUFFER_LIMIT)
        entry->deltas.pop_front();
    
    entry->deltas.push_back(Delta_Address);
    entry->Last_Address = stat.mem_addr;
}



static DCPT_Entry* Create_New_Entry(AccessStat stat)
{
    DCPT_Entry *newentry = new DCPT_Entry(stat.pc);
    newentry->pc = stat.pc;
    newentry->Last_Address = stat.mem_addr;
    newentry->Last_Prefetch = 0;
    newentry->deltas.push_front(1);
    return newentry;
}

static void Store_New_Entry(DCPT_Entry *entry)
{
    static int Number_Entries = 0;
    //Check whether buffer is full or not
    if(Number_Entries < ENTRY_BUFFER_LIMIT)
        ++Number_Entries;
    //Pop the oldest entry
    else
        List_of_Entries.erase(List_of_Entries.begin());

    //Create an entry holding stat values
    List_of_Entries.push_back(entry);
}