#pragma once

#if 0

class ICriticalSection
{
	public:
		ICriticalSection()	{ InitializeCriticalSection(&critSection); }
		~ICriticalSection()	{ DeleteCriticalSection(&critSection); }

		void	Enter(void)		{ EnterCriticalSection(&critSection); }
		void	Leave(void)		{ LeaveCriticalSection(&critSection); }
		bool	TryEnter(void)	{ return TryEnterCriticalSection(&critSection) != 0; }

	private:
		CRITICAL_SECTION	critSection;
};

class ScopedLock
{
public:
	ScopedLock(ICriticalSection& critSection) : m_critSection(critSection)
	{
		m_critSection.Enter();
	}

	~ScopedLock()
	{
		m_critSection.Leave();
	}

private:
	ICriticalSection& m_critSection;
};

#else

class ICriticalSection
{
	DWORD	owningThread;
	DWORD	enterCount;

public:
	ICriticalSection() : owningThread(0), enterCount(0) {}

	void Enter()
	{
		DWORD currThread = GetCurrentThreadId();
		if (owningThread == currThread)
		{
			enterCount++;
			return;
		}
		if (InterlockedCompareExchange(&owningThread, currThread, 0))
		{
			DWORD fastIdx = 10000;
			do {
				Sleep(--fastIdx >> 0x1F);
			} while (InterlockedCompareExchange(&owningThread, currThread, 0));
		}
		enterCount = 1;
	}

	__forceinline void Leave()
	{
		if (!--enterCount)
			owningThread = 0;
	}
};

class CriticalSection : public CRITICAL_SECTION
{
public:
	CriticalSection() { InitializeCriticalSection(this); }
	~CriticalSection() { DeleteCriticalSection(this); }

	void Enter() { EnterCriticalSection(this); }
	void Leave() { LeaveCriticalSection(this); }
	bool TryEnter() { return TryEnterCriticalSection(this) != 0; }
};

class PrimitiveCS
{
	DWORD		m_owningThread;

public:
	PrimitiveCS() : m_owningThread(0) {}

	PrimitiveCS* Enter();
	__forceinline void Leave() { m_owningThread = 0; }
};

class LightCS
{
	UInt32	owningThread = 0;
	UInt32	enterCount = 0;

public:
	void Enter();
	__forceinline void Leave()
	{
		if (!--enterCount)
			owningThread &= 0;
	}
};

template <typename T_CS> class ScopedLock
{
	T_CS* cs;

public:
	ScopedLock(T_CS* _cs) : cs(_cs) { cs->Enter(); }
	~ScopedLock() { cs->Leave(); }
};

typedef ScopedLock<CriticalSection> ScopedCS;
typedef ScopedLock<PrimitiveCS> ScopedPrimitiveCS;
typedef ScopedLock<LightCS> ScopedLightCS;

#endif