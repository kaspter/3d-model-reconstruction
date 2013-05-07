
#include "stdafx.h"
#include "OrphanTask.h"

namespace OrphanTask
{
	namespace
	{
		std::list<HANDLE> _orphanRegistry;

		class Mutex
		{
			CRITICAL_SECTION _crit_section;
		public:
			Mutex()  { InitializeCriticalSection(&_crit_section); }
			~Mutex() { DeleteCriticalSection(&_crit_section);     }
			
			VOID Lock()   { EnterCriticalSection(&_crit_section); }
			VOID Unlock() { LeaveCriticalSection(&_crit_section); }
		} _mutex_object;
	}

	BOOL submit(LPTHREAD_START_ROUTINE lpThreadRoutine, LPVOID lpThreadParam)
	{
		HANDLE orphanThread = CreateThread(NULL, 0, lpThreadRoutine, lpThreadParam, CREATE_SUSPENDED, NULL);
		if (orphanThread == NULL) return FALSE;
		
		_mutex_object.Lock();
	
		_orphanRegistry.push_back(orphanThread);
		ResumeThread(orphanThread);

		_mutex_object.Unlock();

		return TRUE;
	}

	DWORD join(BOOL bAll)
	{
		std::vector<HANDLE> handles(_orphanRegistry.begin(), _orphanRegistry.end());
		DWORD retResult = WaitForMultipleObjects(handles.size(), handles.data(), bAll, INFINITE);

		if (retResult != WAIT_FAILED)
		{
			_mutex_object.Lock();

			if (bAll)
			{
				std::for_each(handles.begin(), handles.end(), [] (HANDLE h) -> void { CloseHandle(h); });
				_orphanRegistry.clear();
			}
			else
			{
				HANDLE handleToClose;
				CloseHandle(handleToClose = handles[retResult - WAIT_OBJECT_0]);
				_orphanRegistry.remove_if([handleToClose] (HANDLE h) -> bool { return h == handleToClose; });
			}

			_mutex_object.Unlock();
		}
		
		return retResult;
	}
}