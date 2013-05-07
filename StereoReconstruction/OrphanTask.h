#pragma once

namespace OrphanTask
{
	BOOL submit(LPTHREAD_START_ROUTINE lpThreadRoutine, LPVOID lpThreadParam);
	DWORD join(BOOL bAll);
}