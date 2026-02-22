import { useState } from "react";

import MultiBlockView from "./MultiBlockView";
import SingleBlockView from "./SingleBlockView";

type TabId = "single" | "multi";

function App() {
  const [activeTab, setActiveTab] = useState<TabId>("single");

  return (
    <div className="flex min-h-0 flex-1 flex-col gap-4">
      <nav className="flex gap-1">
        <button
          className={`rounded-t-lg px-4 py-2 text-sm font-medium transition-colors ${
            activeTab === "single"
              ? "border border-b-0 border-[#333] bg-[#1d1d1d] text-white"
              : "border border-transparent bg-transparent text-[#888] hover:text-[#bbb]"
          }`}
          onClick={() => setActiveTab("single")}
        >
          Single Block
        </button>
        <button
          className={`rounded-t-lg px-4 py-2 text-sm font-medium transition-colors ${
            activeTab === "multi"
              ? "border border-b-0 border-[#333] bg-[#1d1d1d] text-white"
              : "border border-transparent bg-transparent text-[#888] hover:text-[#bbb]"
          }`}
          onClick={() => setActiveTab("multi")}
        >
          Multi Block
        </button>
      </nav>

      {activeTab === "single" ? <SingleBlockView /> : <MultiBlockView />}
    </div>
  );
}

export default App;
