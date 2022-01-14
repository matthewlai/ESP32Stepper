#ifndef CRASH_H
#define CRASH_H

void CrashHandler(const String& msg);

#define CRASH_AND_BURN(msg) CrashHandler(msg);

#endif  // CRASH_H
