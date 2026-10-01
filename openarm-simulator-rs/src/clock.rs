use std::time::{Duration, Instant};

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("clock overflow")]
    Overflow,
    #[error("pause the clock before advancing")]
    Running,
}

#[derive(Default)]
pub struct Clock {
    // Time accumulated before the current running interval, including manual advances.
    accumulated: Duration,
    running_since: Option<Instant>,
}

impl Clock {
    pub fn paused(&self) -> bool {
        self.running_since.is_none()
    }

    pub fn unpause(&mut self) {
        self.running_since.get_or_insert_with(Instant::now);
    }

    pub fn pause(&mut self) -> Result<(), Error> {
        self.accumulated = self.elapsed()?;
        self.running_since = None;
        Ok(())
    }

    /// Read simulation time since startup or reset.
    pub fn elapsed(&self) -> Result<Duration, Error> {
        match self.running_since {
            Some(start) => self
                .accumulated
                .checked_add(start.elapsed())
                .ok_or(Error::Overflow),
            None => Ok(self.accumulated),
        }
    }

    pub fn advance(&mut self, duration: Duration) -> Result<(), Error> {
        if !self.paused() {
            return Err(Error::Running);
        }
        self.accumulated = self
            .accumulated
            .checked_add(duration)
            .ok_or(Error::Overflow)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pause_unpause_and_manual_advances_share_one_timeline() {
        let mut clock = Clock::default();
        assert!(clock.paused());
        assert_eq!(clock.elapsed(), Ok(Duration::ZERO));
        let manual = Duration::from_nanos(12);
        clock.advance(manual).unwrap();
        clock.unpause();
        let start = clock.running_since.unwrap();
        assert!(!clock.paused());
        clock.unpause();
        assert_eq!(clock.running_since, Some(start));
        assert_eq!(clock.advance(Duration::from_nanos(1)), Err(Error::Running));
        let elapsed = clock.elapsed().unwrap();
        assert!(elapsed >= manual);
        assert!(elapsed <= manual + start.elapsed());
        clock.pause().unwrap();
        let paused = clock.elapsed().unwrap();
        assert!(clock.paused());
        assert!(paused >= elapsed);
        assert!(paused <= manual + start.elapsed());
        clock.pause().unwrap();
        assert_eq!(clock.elapsed(), Ok(paused));
        clock.advance(Duration::from_nanos(1)).unwrap();
        let advanced = paused + Duration::from_nanos(1);
        assert_eq!(clock.elapsed(), Ok(advanced));
        clock.unpause();
        assert!(clock.elapsed().unwrap() >= advanced);
    }

    #[test]
    fn overflow_leaves_clock_unchanged() {
        let mut clock = Clock::default();
        clock.advance(Duration::MAX).unwrap();
        assert_eq!(clock.advance(Duration::from_nanos(1)), Err(Error::Overflow));
        assert_eq!(clock.elapsed(), Ok(Duration::MAX));
        let start = Instant::now() - Duration::from_secs(1);
        clock.running_since = Some(start);
        assert_eq!(clock.elapsed(), Err(Error::Overflow));
        assert_eq!(clock.pause(), Err(Error::Overflow));
        assert!(!clock.paused());
        assert_eq!(clock.accumulated, Duration::MAX);
        assert_eq!(clock.running_since, Some(start));
    }
}
