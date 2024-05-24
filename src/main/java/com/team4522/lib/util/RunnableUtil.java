package com.team4522.lib.util;

import java.util.concurrent.atomic.AtomicBoolean;

public class RunnableUtil {

    private static class RunOnce {
        private final AtomicBoolean hasRun = new AtomicBoolean(false);

        public void runOnce(Runnable runnable) {
            if (hasRun.compareAndSet(false, true)) {
                runnable.run();
            }
        }

        public void runOnceWhen(Runnable runnable, boolean condition) {
            if (!hasRun.get() && condition && hasRun.compareAndSet(false, true)) {
                runnable.run();
            }
        }

        public void reset() {
            hasRun.set(false);
        }
    }

    private static class RunUntil {
        private final AtomicBoolean shouldStop = new AtomicBoolean(false);
    
        public void runUntil(Runnable runnable, boolean stopCondition) {
            if (!shouldStop.get() && stopCondition) {
                runnable.run();
                shouldStop.compareAndSet(false, true);
            }
        }
    
        public void reset() {
            shouldStop.set(false);
        }
    
        public boolean hasStopped(){
            return shouldStop.get();
        }
    }

    public static RunOnce runOnce(){
        return new RunOnce();
    }

    public static RunUntil runUntil(){
        return new RunUntil();
    }
}
