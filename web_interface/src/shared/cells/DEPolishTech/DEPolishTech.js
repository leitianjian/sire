import React, { Component } from 'react'
import { ConfigConsumer } from 'antd/lib/config-provider'
import Icon from 'antd/lib/icon'
import Row from 'antd/lib/row'
import Col from 'antd/lib/col'
import Input from 'antd/lib/input'
import Button from 'antd/lib/button'
import Select from 'antd/lib/select'
import { connect } from 'react-redux'
import Table from 'antd/lib/table'
import Form from 'antd/lib/form'
import Modal from 'antd/lib/modal'
import Upload from 'antd/lib/upload'

import CellBase from '../CellBase'
import {
  loadPrograms,
  updateProgram,
  createProgram,
  dispatchFetchObjPictureList,
  dispatchUpdateObjPictureList,
  sendCmd
} from '../../../state/actions'

// 进行中英文切换
import zhCN from '../locale/zhCN'
import { IntlProvider, FormattedMessage } from 'react-intl'
import enUS from '../locale/enUS'
const messages = {}
messages.chinese = zhCN
messages.english = enUS

const Option = Select.Option

const slaveFormItemLayout = {
  labelCol: {
    span: 10
  },
  wrapperCol: {
    span: 14
  }
}

class CreateTechModalRaw extends Component {
  formRef = React.createRef()
  state = {
    force_running: false,
    pro_name: null,
    is_change_tool: false
  }

  handleSubmit = (e) => {
    e.preventDefault()

    this.formRef.current.validateFields((err, data) => {
      if (err) {
        return
      }
      this.props.okModal(data)
      this.props.closeModal()
    })
  }

  renderFormItemInput (
    prefixCls,
    key,
    label,
    initialValue,
    rules,
    span,
    required
  ) {
    return (
      <Col span={span || 7} offset={1} pull={1} key={key}>
        <Form.Item
          {...slaveFormItemLayout}
          label={label}
          hasFeedback
          name={`${key}`}
          initialValue={initialValue}
          rules={[
            {
              required: !!required,
              message: `请输入${label}`
            }
          ]}
        >
          <Input placeholder={`${label}`} size={'small'} />
        </Form.Item>
      </Col>
    )
  }

  render () {
    const { prefixCls } = this.props
    const program_list = []
    if (Object.keys(this.props.programs).length > 0) {
      Object.keys(this.props.programs).forEach((program_key) => {
        const program = this.props.programs[program_key]
        program.files.forEach((file) => {
          const name = program.name + '.' + file.name.split('.')[0]
          if (
            program_list.find((value) => {
              return value.key === name
            }) === undefined
          ) { program_list.push({ key: name }) }
        })
      })
    }

    return (
      <IntlProvider>
        <Modal
          width="800px"
          title={<FormattedMessage id="添加工艺" />}
          okText={<FormattedMessage id="创建" />}
          cancelText={<FormattedMessage id="取消" />}
          onOk={this.handleSubmit}
          visible={this.props.visible || this.state.force_running}
          onCancel={this.props.closeModal}
        >
          <Form ref={this.formRef}>
            <Button
              type={'primary'}
              disabled={this.state.force_running}
              onClick={() => {
                this.props.sendCmd('tmode --mode=1')
                this.props.sendCmd('mvJoint')
                this.setState({ force_running: true })
              }}
            >
              <FormattedMessage id="拖动示教" />
            </Button>
            <Button
              type={'primary'}
              disabled={!this.state.force_running}
              onClick={() => {
                const name = `drag_${Date.now()}`
                this.props.sendCmd('FCStop')

                this.setState({ pro_name: name })
                this.formRef.current.setFieldsValue({
                  program: 'drag.' + name
                })

                setTimeout(() => {
                  this.props.sendCmd(
                    `genfun --mainfun=drag --subfun=${name} --pos=0.01`,
                    'genfun',
                    () => {
                      this.setState({ force_running: false })
                    }
                  )

                  setTimeout(() => {
                    this.props.dspLoadPrograms()
                  }, 1000)
                }, 3000)
              }}
            >
              <FormattedMessage id="结束拖动" />
            </Button>
            <Row>
              <Col span={7} pull={1} offset={1}>
                <Form.Item
                  label="图示"
                  {...slaveFormItemLayout}
                  name={'img'}
                  rules={[
                    {
                      type: 'string',
                      required: true,
                      message: '请选择图示'
                    }
                  ]}
                >
                  <Select
                    defaultValue={
                      program_list.length > 0 ? program_list[0] : null
                    }
                    style={{ width: 120 }}
                    onChange={(e) => {}}
                  >
                    {this.props.detechFileList.map((d) => {
                      return <Option value={`${d.url}`}>{d.name}</Option>
                    })}
                  </Select>
                </Form.Item>
              </Col>
              <Col span={7} pull={1} offset={1}>
                <Form.Item
                  label="程序号"
                  {...slaveFormItemLayout}
                  name={'program'}
                  rules={[
                    {
                      type: 'string',
                      required: true,
                      message: '请选择程序'
                    }
                  ]}
                >
                  <Input
                    placeholder={'请拖动程序'}
                    disabled={true}
                    size={'small'}
                  />
                </Form.Item>
              </Col>
              <Col span={7} pull={1} offset={1}>
                <Form.Item
                  label="描述"
                  {...slaveFormItemLayout}
                  name={'description'}
                  rules={[
                    {
                      type: 'string',
                      required: true,
                      message: '请输入程序名称'
                    }
                  ]}
                >
                  <Input size={'small'} />
                </Form.Item>
              </Col>
            </Row>
            <Row>
              {[
                ['height', '高度(mm)', null, [{ type: 'integer' }]],
                ['thickness', '厚度(mm)', null, [{ type: 'integer' }]],
                ['length', '长度(mm)', null, [{ type: 'integer' }]]
              ].map((d) => this.renderFormItemInput(prefixCls, ...d, 7, true))}
            </Row>
            <Row>
              {[
                [
                  'translational_speed',
                  '进给速度(mm/s)',
                  null,
                  [{ type: 'integer' }]
                ],
                ['rotational_speed', '转速(rpm)', null, [{ type: 'integer' }]],
                ['polish_method', '打磨方式', null, [{ type: 'string' }]]
              ].map((d) => this.renderFormItemInput(prefixCls, ...d, 7, true))}
            </Row>
            <Row>
              {[
                ['tool_num', '刀具序号', null, [{ type: 'integer' }]],
                ['tool_name', '刀具名称', null, [{ type: 'string' }]],
                ['tool_pic', '刀具照片', null, [{ type: 'string' }]]
              ].map((d) => this.renderFormItemInput(prefixCls, ...d, 7, true))}
            </Row>
            <Row>
              {[
                ['tool_length', '刀具长度', null, [{ type: 'integer' }]],
                ['tool_radius', '刀具直径', null, [{ type: 'integer' }]],
                ['material', '材料牌号', null, [{ type: 'string' }]]
              ].map((d) => this.renderFormItemInput(prefixCls, ...d, 7, true))}
            </Row>
          </Form>
        </Modal>
      </IntlProvider>
    )
  }
}

class PicturesWall extends React.Component {
  state = {
    previewVisible: false,
    previewImage: ''
  }

  handleCancel = () => this.setState({ previewVisible: false })

  handlePreview = (file) => {
    this.setState({
      previewImage: file.url || file.thumbUrl,
      previewVisible: true
    })
  }

  handleChange = ({ fileList }) => {
    console.log('filelist:', fileList)
    this.props.updateObjPictureList(fileList)
  }

  render () {
    const { previewVisible, previewImage } = this.state
    const uploadButton = (
      <div>
        <Icon type="plus" />
        <div className="ant-upload-text">Upload</div>
      </div>
    )

    return (
      <div className="clearfix">
        <Upload
          action="/api/obj_picture"
          listType="picture-card"
          fileList={this.props.fileList}
          onPreview={this.handlePreview}
          onChange={this.handleChange}
        >
          {uploadButton}
        </Upload>
        <Modal
          visible={previewVisible}
          footer={null}
          onCancel={this.handleCancel}
        >
          <img alt="example" style={{ width: '100%' }} src={previewImage} />
        </Modal>
      </div>
    )
  }
}

class DEPolishTech extends CellBase {
  state = {
    creating_tech: false,
    editing_tech: false,
    selectedRowKeys: []
  }

  componentDidMount () {
    this.props.fetchObjPictureList()
  }

  renderFooter = (prefixCls) => {
    return <div />
  }

  renderHeader = (prefixCls) => {
    return (
      <div className={`${prefixCls}-header`}>
        <Row>
          <PicturesWall
            fileList={this.props.detechFileList}
            updateObjPictureList={this.props.updateObjPictureList}
          />
        </Row>
        <CreateTechModalRaw
          prefix={`${prefixCls}-header`}
          visible={this.state.creating_tech}
          programs={this.props.programs}
          detechFileList={this.props.detechFileList}
          sendCmd={this.props.sendCmd}
          dspLoadPrograms={this.props.dspLoadPrograms}
          okModal={(data) => {
            const options = this.getOptions()
            if (!options.tech_data) options.tech_data = []

            const datas = options.tech_data
            if (this.state.selectedRowKeys.length === 1) {
              datas.splice(this.state.selectedRowKeys[0], 0, data)
              let i = 1
              datas.forEach((d) => {
                d.number = i
                i++
              })
            } else {
              data.number = options.tech_data.length + 1
              datas.push(data)
            }

            this.updateOptions({ ...options, tech_data: datas })
          }}
          closeModal={() => {
            this.setState({ creating_tech: false })
            console.log('close')
          }}
        />
        <Button
          type={'primary'}
          onClick={() => {
            this.setState({ creating_tech: true })
          }}
        >
          <FormattedMessage id="新建" />
        </Button>
        <Button type={'primary'} onClick={() => {}}>
          <FormattedMessage id="编辑" />
        </Button>
        <Button
          type={'primary'}
          onClick={() => {
            const options = this.getOptions()
            if (!options.tech_data) options.tech_data = []
            const datas = options.tech_data
            const select_datas = this.state.selectedRowKeys

            if (select_datas.length > 0) {
              select_datas.sort()
              for (let i = select_datas.length - 1; i >= 0; i--) {
                datas.splice(select_datas[i], 1)
              }
            }

            for (let i = 0; i < datas.length; ++i) {
              datas[i].number = i + 1
            }

            this.updateOptions({ ...options, tech_data: datas })
          }}
        >
          <FormattedMessage id="删除" />
        </Button>
        <Button
          type={'primary'}
          onClick={() => {
            let i = 1
            let name = 'program' + i
            while (
              Object.keys(this.props.programs).findIndex((e) => e === name) !==
              -1
            ) {
              i++
              name = 'program' + i
            }
            this.props.createProgram({ name })
            const newProgram = {
              files: [],
              name,
              path: `/program/${name}`
            }

            const options = this.getOptions()
            if (!options.tech_data) options.tech_data = []

            // make main function //
            let pro_content =
              '<xml xmlns="https://developers.google.com/blockly/xml">'
            pro_content +=
              '<block type="main" id="2" x="110" y="30"><statement name="body">'

            i = 0
            options.tech_data.forEach((data) => {
              const names = data.program.split('.')
              const program = this.props.programs[names[0]]

              const pro_idx = program.files.findIndex((e) => {
                return e.name === names[1] + '.pro'
              })
              if (
                pro_idx !== -1 &&
                newProgram.files.findIndex(
                  (e) => e.name === names[1] + '.pro'
                ) === -1
              ) {
                const content = String(
                  this.props.programs[names[0]].files[pro_idx].content
                )
                newProgram.files.push({
                  name: names[1] + '.pro',
                  content
                })
              }
              const dat_idx = program.files.findIndex((e) => {
                return e.name === names[1] + '.dat'
              })
              if (
                dat_idx !== -1 &&
                newProgram.files.findIndex(
                  (e) => e.name === names[1] + '.dat'
                ) === -1
              ) {
                const content = String(
                  this.props.programs[names[0]].files[dat_idx].content
                )
                newProgram.files.push({
                  name: names[1] + '.dat',
                  content
                })
              }
              const aris_idx = program.files.findIndex((e) => {
                return e.name === names[1] + '.aris'
              })
              if (
                aris_idx !== -1 &&
                newProgram.files.findIndex(
                  (e) => e.name === names[1] + '.aris'
                ) === -1
              ) {
                const content = String(
                  this.props.programs[names[0]].files[aris_idx].content
                )
                newProgram.files.push({
                  name: names[1] + '.aris',
                  content
                })
              }

              if (i === 0) {
                pro_content +=
                  '<block type="set_speed" id="3"><field name="speed">' +
                  names[1] +
                  '.' +
                  'v1' +
                  '</field><field name="SPEED0">' +
                  10 +
                  '</field><field name="SPEED1">' +
                  data.translational_speed / 1000 +
                  '</field><field name="SPEED2">' +
                  60 +
                  '</field><field name="SPEED3">0</field><field name="SPEED4">0</field><next><block type="set_zone" id="4"><field name="zone">' +
                  names[1] +
                  '.' +
                  'z1' +
                  '</field><field name="DIS">10</field><field name="PER">20</field><next><block type="call" id="5"><field name="func_name">' +
                  names[1] +
                  '.' +
                  'f' +
                  '</field>'
              } else {
                pro_content +=
                  '<next><block type="set_speed" id="3"><field name="speed">' +
                  names[1] +
                  '.' +
                  'v1' +
                  '</field><field name="SPEED0">' +
                  10 +
                  '</field><field name="SPEED1">' +
                  data.translational_speed / 1000 +
                  '</field><field name="SPEED2">' +
                  60 +
                  '</field><field name="SPEED3">0</field><field name="SPEED4">0</field><next><block type="set_zone" id="4"><field name="zone">' +
                  names[1] +
                  '.' +
                  'z1' +
                  '</field><field name="DIS">10</field><field name="PER">20</field><next><block type="call" id="5"><field name="func_name">' +
                  names[1] +
                  '.' +
                  'f' +
                  '</field>'
              }

              i++
            })

            while (i - 1 > 0) {
              pro_content += '</block></next></block></next></block></next>'
              i--
            }
            pro_content += '</block></next></block></next></block>'
            pro_content += '</statement></block>'
            pro_content += '</xml>'

            newProgram.files.push({ name: 'main.pro', content: pro_content })
            newProgram.files.push({
              name: 'main.dat',
              content:
                '<xml xmlns="https://developers.google.com/blockly/xml"/>'
            })
            newProgram.files.push({ name: 'main.aris', content: '' })

            this.props.updateProgram(newProgram.name, newProgram)
          }}
        >
          <FormattedMessage id="生成程序" />
        </Button>
      </div>
    )
  }

  renderTable = (prefixCls) => {
    const columns = [
      {
        title: '序号',
        dataIndex: 'number',
        key: 'number',
        align: 'center'
      },
      {
        title: '图示',
        dataIndex: 'img',
        key: 'img',
        align: 'center',
        render: (im) => {
          return <img src={`/${im}`} width="50" height="50" />
        }
      },
      {
        title: '描述',
        dataIndex: 'description',
        key: 'description',
        align: 'left'
      },
      {
        title: '程序号',
        dataIndex: 'program',
        key: 'program',
        align: 'center'
      },
      {
        title: '高度',
        dataIndex: 'height',
        key: 'height',
        align: 'center'
      },
      {
        title: '厚度',
        dataIndex: 'thickness',
        key: 'thickness',
        align: 'center'
      },
      {
        title: '长度',
        dataIndex: 'length',
        key: 'length',
        align: 'center'
      },
      {
        title: '刀具序号',
        dataIndex: 'tool_num',
        key: 'tool_num',
        align: 'center'
      },
      {
        title: '刀具名称',
        dataIndex: 'tool_name',
        key: 'tool_name',
        align: 'center'
      },
      {
        title: '刀具照片',
        dataIndex: 'tool_pic',
        key: 'tool_pic',
        align: 'center'
      },
      {
        title: '刀具长度',
        dataIndex: 'tool_length',
        key: 'tool_length',
        align: 'center'
      },
      {
        title: '刀具直径',
        dataIndex: 'tool_radius',
        key: 'tool_radius',
        align: 'center'
      },
      {
        title: '材料牌号',
        dataIndex: 'material',
        key: 'material',
        align: 'center'
      },
      {
        title: '进给速度',
        dataIndex: 'translational_speed',
        key: 'translational_speed',
        align: 'center'
      },
      {
        title: '转速',
        dataIndex: 'rotational_speed',
        key: 'rotational_speed',
        align: 'center'
      },
      {
        title: '打磨方式',
        dataIndex: 'polish_method',
        key: 'polish_method',
        align: 'center'
      }
    ]

    const rowSelection = {
      selectedRowKeys: this.state.selectedRowKeys,
      onChange: (selectedRowKeys, selectedRows) => {
        this.setState({ selectedRowKeys })
      },
      getCheckboxProps: (record) => ({
        disabled: record.name === 'Disabled User',
        name: record.name
      })
    }

    const idx = this.getOptions().tech_data
      ? this.getOptions().tech_data.findIndex((d) => {
        if (!!d.program && !!this.props.currentFile) {
          const names1 = d.program.split('.')
          const names2 = this.props.currentFile.split('.')
          return names1[1] === names2[0]
        } else {
          return false
        }
      })
      : null

    console.log('selected id:', idx)

    const rowSelection_auto = {
      selectedRowKeys: [idx]
    }

    return (
      <div>
        <Table
          className={`${prefixCls}-table`}
          rowSelection={
            this.props.stateCode === 400 ? rowSelection_auto : rowSelection
          }
          dataSource={this.getOptions().tech_data}
          columns={columns}
          size={'small'}
          bordered
          pagination={false}
        />
      </div>
    )
  }

  renderContent = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props
    const prefixCls = getPrefixCls('de-tech', customizePrefixCls)
    return (
      <div className={`${prefixCls}`}>
        <Row>{this.renderHeader(prefixCls)}</Row>
        <Row>{this.renderTable(prefixCls)}</Row>
        <Row>{this.renderFooter(prefixCls)}</Row>
      </div>
    )
  }

  render () {
    const { localestate } = this.props
    return (
      <IntlProvider locale={localestate} messages={messages[localestate]}>
        <ConfigConsumer>{this.renderContent}</ConfigConsumer>
      </IntlProvider>
    )
  }
}

function mapStateToProps (state) {
  return {
    programs: state.robot.programs,
    detechFileList: state.robot.fetchingObjPictureListResult,
    stateCode:
      state.robot.resultsByChannel.get &&
      state.robot.resultsByChannel.get.state_code,
    currentFile:
      state.robot.resultsByChannel.get && state.robot.resultsByChannel.get.file
  }
}

function mapDispatchToProps (dispatch) {
  return {
    createProgram: (...args) => dispatch(createProgram(...args)),
    updateProgram: (...args) => dispatch(updateProgram(...args)),
    fetchObjPictureList: (...args) =>
      dispatch(dispatchFetchObjPictureList(...args)),
    updateObjPictureList: (...args) =>
      dispatch(dispatchUpdateObjPictureList(...args)),
    sendCmd: (...args) => dispatch(sendCmd(...args)),
    dspLoadPrograms: (...args) => dispatch(loadPrograms(...args))
  }
}

const ConnectedDEPolishTech = connect(
  mapStateToProps,
  mapDispatchToProps
)(DEPolishTech)

ConnectedDEPolishTech.NAME = '戴屹打磨工艺'
export default ConnectedDEPolishTech
