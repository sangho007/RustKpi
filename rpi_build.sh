cd /rust_ws/RustKpi

cargo build --release && cp ./target/aarch64-unknown-linux-gnu/release/RustKpi .

git add .
git commit -m ".."
git push origin master

echo task done!!!
