import React, { useState } from "react";
import ReactECharts from "echarts-for-react";
import { withSize } from "react-sizeme";

interface DataItem {
  label: string;
  data: number[];
}

const ChartDisplay: React.FC = () => {
  const [chartData, setChartData] = useState<DataItem[]>([]);
  const [activeSeries, setActiveSeries] = useState<Set<string>>(new Set());

  const calculateForceMagnitude = (force: number[]): number => {
    return Math.sqrt(force.reduce((sum, val) => sum + val * val, 0));
  };

  // 处理文件上传
  const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
  
    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const parsedJson = JSON.parse(e.target?.result as string);
  
        // 1. 提取 contact_info 数组
        const contactInfoArray: any[] = parsedJson.contact_info;
        if (!Array.isArray(contactInfoArray)) {
          throw new Error("contact_info 不存在或不是数组");
        }
  
        const processedData: DataItem[] = [];
  
        // 2. 初始化 dataMap
        const dataMap: { [key: string]: number[] } = {};
  
        // 3. 遍历 contact_info 逐时间片处理
        contactInfoArray.forEach((timeSlice, timeIndex) => {
          if (Array.isArray(timeSlice)) {
            timeSlice.forEach((contact: any) => {
              const key = `${contact.partId_A}-${contact.partId_B}`;
              if (!dataMap[key]) {
                dataMap[key] = new Array(contactInfoArray.length).fill(0);
              }
              dataMap[key][timeIndex] = calculateForceMagnitude(contact.contact_force);
            });
          } else {
            // timeSlice 为 null，所有键对应时间点力为 0
            Object.keys(dataMap).forEach((key) => {
              dataMap[key][timeIndex] = 0;
            });
          }
        });
  
        // 4. 构建 processedData
        Object.keys(dataMap).forEach((key) => {
          processedData.push({
            label: `Part ${key}`,
            data: dataMap[key],
          });
        });
  
        // 5. 更新状态
        setChartData(processedData);
        setActiveSeries(new Set(processedData.map((item) => item.label))); // 默认全部显示
      } catch (error) {
        console.error("JSON 解析失败:", error);
        alert("上传的 JSON 格式不正确！");
      }
    };
    reader.readAsText(file);
  };

  // 处理图例点击事件
  const toggleSeries = (label: string) => {
    setActiveSeries((prev) => {
      const newSet = new Set(prev);
      if (newSet.has(label)) {
        newSet.delete(label);
      } else {
        newSet.add(label);
      }
      return newSet;
    });
  };

  // 配置 ECharts 图表
  const options = {
    title: { text: "数据变化趋势", left: "center" },
    tooltip: { trigger: 'axis' as const },
    legend: {
      data: chartData.map((item) => item.label),
      bottom: 0,
      selected: Object.fromEntries(chartData.map((item) => [item.label, activeSeries.has(item.label)])),
      textStyle: { fontSize: 12 },
      icon: "circle",
    },
    xAxis: { type: "category" as const}, 
    yAxis: { type: "value" as const },
    series: chartData
      .filter((item) => activeSeries.has(item.label))
      .map((item) => ({
        name: item.label,
        type: "line" as const,
        data: item.data,
        smooth: true,
      })),
  };

  return (
    <div style={{ textAlign: "center", padding: "20px" }}>
      {/* 上传按钮 */}
      <input type="file" accept=".json" onChange={handleFileUpload} />

      {/* 折线图 */}
      {chartData.length > 0 && (
        <ReactECharts option={options} style={{ width: "900px", height: "400px" }} />
      )}

      {/* 图例点击控制 */}
      {chartData.length > 0 && (
        <div style={{ marginTop: "10px" }}>
          {chartData.map((item) => (
            <button
              key={item.label}
              onClick={() => toggleSeries(item.label)}
              style={{
                margin: "5px",
                padding: "5px 10px",
                borderRadius: "5px",
                border: "1px solid #ccc",
                backgroundColor: activeSeries.has(item.label) ? "#007bff" : "#ccc",
                color: "#fff",
                cursor: "pointer",
              }}
            >
              {item.label}
            </button>
          ))}
        </div>
      )}
    </div>
  );
};

// 注册组件，符合你的项目格式
const chartDisplay = withSize({ monitorHeight: true, refreshRate: 30 })(ChartDisplay) as React.ComponentType<{}> & {
  NAME: string;
};

chartDisplay.NAME = "接触力展示";
export default chartDisplay;