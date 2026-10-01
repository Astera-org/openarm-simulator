//! Shared, checksum-verified SDK/model downloads in the user's platform cache.
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    path::{Path, PathBuf},
    process::Command,
};

pub fn cache(name: &str) -> PathBuf {
    for variable in ["HOME", "XDG_CACHE_HOME"] {
        println!("cargo:rerun-if-env-changed={variable}");
    }
    directories::BaseDirs::new()
        .expect("could not locate user cache directory")
        .cache_dir()
        .join(name)
}

pub fn fetch(cache: &Path, url: &str, checksum: &str, members: &[&str]) -> PathBuf {
    fs::create_dir_all(cache).expect("create dependency cache");
    let destination = cache.join(checksum);
    let lock = File::options()
        .create(true)
        .append(true)
        .open(cache.join(format!("{checksum}.lock")))
        .expect("open dependency cache lock");
    lock.lock().expect("lock dependency cache");
    if destination.is_dir() {
        return destination;
    }
    eprintln!("Downloading {url}");
    let temporary = tempfile::tempdir_in(cache).expect("create download directory");
    let archive = temporary.path().join("archive.tar.gz");
    let status = Command::new("curl")
        .args([
            "--fail",
            "--location",
            "--silent",
            "--show-error",
            "--retry",
            "3",
        ])
        .arg(url)
        .arg("--output")
        .arg(&archive)
        .status()
        .expect("run curl to download dependency");
    assert!(status.success(), "download failed: {url}");
    assert_eq!(
        format!("{:x}", Sha256::digest(fs::read(&archive).unwrap())),
        checksum,
        "checksum mismatch: {url}"
    );
    let unpacked = temporary.path().join("unpacked");
    fs::create_dir(&unpacked).unwrap();
    let status = Command::new("tar")
        .args(["--extract", "--gzip", "--strip-components=1", "--file"])
        .arg(&archive)
        .arg("--directory")
        .arg(&unpacked)
        .args(members)
        .status()
        .expect("run tar to extract dependency");
    assert!(status.success(), "extract failed: {url}");
    // Only publish complete, verified assets; concurrent worktrees share the lock.
    fs::rename(unpacked, &destination).expect("publish dependency cache entry");
    destination
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn shared_verified_assets() {
        let temporary = tempfile::tempdir().unwrap();
        let base = temporary.path();
        let cache = base.join("cache");
        fs::create_dir(base.join("payload")).unwrap();
        fs::write(base.join("payload/value"), "verified fixture").unwrap();
        let archive = base.join("fixture.tar.gz");
        assert!(
            Command::new("tar")
                .current_dir(base)
                .args(["-czf", "fixture.tar.gz", "payload"])
                .status()
                .unwrap()
                .success()
        );
        let checksum = format!("{:x}", Sha256::digest(fs::read(&archive).unwrap()));
        let url = format!("file://{}", archive.display());
        let paths = thread::scope(|scope| {
            let a = scope.spawn(|| fetch(&cache, &url, &checksum, &[]));
            let b = scope.spawn(|| fetch(&cache, &url, &checksum, &[]));
            [a.join().unwrap(), b.join().unwrap()]
        });
        assert_eq!(paths[0], paths[1]);
        assert_eq!(
            fs::read_to_string(paths[0].join("value")).unwrap(),
            "verified fixture"
        );
        // Evicted entries are downloaded and extracted again.
        fs::remove_dir_all(&paths[0]).unwrap();
        let restored = fetch(&cache, &url, &checksum, &[]);
        assert_eq!(
            fs::read_to_string(restored.join("value")).unwrap(),
            "verified fixture"
        );
        let wrong_checksum = "0".repeat(64);
        assert!(std::panic::catch_unwind(|| fetch(&cache, &url, &wrong_checksum, &[])).is_err());
        assert!(!cache.join(wrong_checksum).exists());
        fs::remove_file(archive).unwrap();
        // Completed entries remain usable without contacting the source.
        assert_eq!(fetch(&cache, &url, &checksum, &[]), paths[0]);
    }
}
