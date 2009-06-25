/*Instance of task t1*/

#include "embUnit.h"
#include "Os.h"

DeclareCounter(Software_Counter1);
DeclareCounter(Software_Counter2);
DeclareScheduleTable(sched1);
DeclareScheduleTable(sched2);

/*test case:test the reaction of the system called with 
 an activation of a task*/
static void test_t1_instance(void)
{
	StatusType result_inst_1, result_inst_2, result_inst_3, result_inst_4, result_inst_5, result_inst_6, result_inst_7, result_inst_8, result_inst_9, result_inst_10, result_inst_11, result_inst_12, result_inst_13, result_inst_14, result_inst_15, result_inst_16, result_inst_17;
	ScheduleTableStatusType ScheduleTableStatusType_inst_1, ScheduleTableStatusType_inst_2, ScheduleTableStatusType_inst_3, ScheduleTableStatusType_inst_4, ScheduleTableStatusType_inst_5, ScheduleTableStatusType_inst_6;
		
	SCHEDULING_CHECK_INIT(1);
	result_inst_1 = GetScheduleTableStatus(sched1, &ScheduleTableStatusType_inst_1);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(1, SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS , ScheduleTableStatusType_inst_1);
	SCHEDULING_CHECK_AND_EQUAL_INT(1,E_OK, result_inst_1);
	
	SCHEDULING_CHECK_INIT(2);
	result_inst_2 = GetScheduleTableStatus(sched2, &ScheduleTableStatusType_inst_2);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(2, SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS , ScheduleTableStatusType_inst_2);
	SCHEDULING_CHECK_AND_EQUAL_INT(2,E_OK, result_inst_2);
	
	SCHEDULING_CHECK_INIT(3);
	result_inst_3 = IncrementCounter(Software_Counter1);
	/*offset = 1 */
	SCHEDULING_CHECK_AND_EQUAL_INT(3,E_OK, result_inst_3);
	
	SCHEDULING_CHECK_INIT(4);
	result_inst_3 = IncrementCounter(Software_Counter2);
	/*offset = 1 */
	SCHEDULING_CHECK_AND_EQUAL_INT(4,E_OK, result_inst_3);
	
	SCHEDULING_CHECK_INIT(5);
	result_inst_4 = IncrementCounter(Software_Counter1);
	/*offset = 2 -> sched1 starts */
	SCHEDULING_CHECK_AND_EQUAL_INT(5,E_OK, result_inst_4);
		
	SCHEDULING_CHECK_INIT(6);
	result_inst_5 = IncrementCounter(Software_Counter2);
	/*offset = 2 -> sched2 starts and event1 can't be set -> errorhook*/
	SCHEDULING_CHECK_AND_EQUAL_INT(7,E_OK, result_inst_5);
	
	SCHEDULING_CHECK_INIT(8);
	result_inst_6 = IncrementCounter(Software_Counter1);
	/*offset = 3 -> t2 starts */
	SCHEDULING_CHECK_AND_EQUAL_INT(9,E_OK, result_inst_6);
	
	SCHEDULING_CHECK_INIT(10);
	result_inst_7 = IncrementCounter(Software_Counter2);
	/*offset = 3 */
	SCHEDULING_CHECK_AND_EQUAL_INT(10,E_OK, result_inst_7);
	
	SCHEDULING_CHECK_INIT(11);
	result_inst_6 = IncrementCounter(Software_Counter1);
	/*offset = 4  */
	SCHEDULING_CHECK_AND_EQUAL_INT(11,E_OK, result_inst_6);
	
	SCHEDULING_CHECK_INIT(12);
	result_inst_7 = IncrementCounter(Software_Counter2);
	/*offset = 4 -> */
	SCHEDULING_CHECK_AND_EQUAL_INT(12,E_OK, result_inst_7);
	
	SCHEDULING_CHECK_INIT(13);
	result_inst_6 = IncrementCounter(Software_Counter1);
	/*offset = 5 -> sched2 restarts */
	SCHEDULING_CHECK_AND_EQUAL_INT(13,E_OK, result_inst_6);
	
	SCHEDULING_CHECK_INIT(14);
	result_inst_7 = IncrementCounter(Software_Counter2);
	/*offset = 5 -> */
	SCHEDULING_CHECK_AND_EQUAL_INT(14,E_OK, result_inst_7);
	
	SCHEDULING_CHECK_INIT(15);
	result_inst_6 = IncrementCounter(Software_Counter1);
	/*offset = 6 -> t2 can't start -> errorhook */
	SCHEDULING_CHECK_AND_EQUAL_INT(16,E_OK, result_inst_6);
	
	SCHEDULING_CHECK_INIT(17);
	result_inst_7 = IncrementCounter(Software_Counter2);
	/*offset = 6 */
	SCHEDULING_CHECK_AND_EQUAL_INT(17,E_OK, result_inst_7);
	
	SCHEDULING_CHECK_INIT(18);
	result_inst_6 = IncrementCounter(Software_Counter1);
	/*offset = 7 */
	SCHEDULING_CHECK_AND_EQUAL_INT(18,E_OK, result_inst_6);
	
	SCHEDULING_CHECK_INIT(19);
	result_inst_7 = IncrementCounter(Software_Counter2);
	/*offset = 7 -> t2 received event1 */
	SCHEDULING_CHECK_AND_EQUAL_INT(20,E_OK, result_inst_7);
		
	SCHEDULING_CHECK_INIT(21);
	result_inst_12 = GetScheduleTableStatus(sched1, &ScheduleTableStatusType_inst_3);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(21, SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS , ScheduleTableStatusType_inst_3);
	SCHEDULING_CHECK_AND_EQUAL_INT(21,E_OK, result_inst_12);
	
	SCHEDULING_CHECK_INIT(22);
	result_inst_13 = GetScheduleTableStatus(sched2, &ScheduleTableStatusType_inst_4);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(22, SCHEDULETABLE_RUNNING_AND_SYNCHRONOUS , ScheduleTableStatusType_inst_4);
	SCHEDULING_CHECK_AND_EQUAL_INT(22,E_OK, result_inst_13);
	
	SCHEDULING_CHECK_INIT(23);
	result_inst_14 = StopScheduleTable(sched1);
	SCHEDULING_CHECK_AND_EQUAL_INT(23,E_OK, result_inst_14);
	
	SCHEDULING_CHECK_INIT(24);
	result_inst_15 = StopScheduleTable(sched2);
	SCHEDULING_CHECK_AND_EQUAL_INT(24,E_OK, result_inst_15);
	
	SCHEDULING_CHECK_INIT(25);
	result_inst_16 = GetScheduleTableStatus(sched1, &ScheduleTableStatusType_inst_5);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(25, SCHEDULETABLE_STOPPED , ScheduleTableStatusType_inst_5);
	SCHEDULING_CHECK_AND_EQUAL_INT(25,E_OK, result_inst_16);
	
	SCHEDULING_CHECK_INIT(26);
	result_inst_17 = GetScheduleTableStatus(sched2, &ScheduleTableStatusType_inst_6);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(26, SCHEDULETABLE_STOPPED , ScheduleTableStatusType_inst_6);
	SCHEDULING_CHECK_AND_EQUAL_INT(26,E_OK, result_inst_17);
		
}

/*create the test suite with all the test cases*/
TestRef AutosarSTSTest_seq1_t1_instance(void)
{
	EMB_UNIT_TESTFIXTURES(fixtures) {
		new_TestFixture("test_t1_instance",test_t1_instance)
	};
	EMB_UNIT_TESTCALLER(AutosarSTSTest,"AutosarSTTest_sequence1",NULL,NULL,fixtures);
	
	return (TestRef)&AutosarSTSTest;
}
